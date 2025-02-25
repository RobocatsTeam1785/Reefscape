package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.function.Consumer;

import com.studica.frc.AHRS;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.SwerveConstants;
import frc.lib.swerve.TalonSwerveModule;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Swerve extends SubsystemBase {
    // components
    // - abstracted
    @Logged protected final TalonSwerveModule flModule = new TalonSwerveModule("FL", SwerveConstants.FL_DRIVE_ID, SwerveConstants.FL_TURN_ID, SwerveConstants.FL_ENCODER_ID);
    @Logged protected final TalonSwerveModule frModule = new TalonSwerveModule("FR", SwerveConstants.FR_DRIVE_ID, SwerveConstants.FR_TURN_ID, SwerveConstants.FR_ENCODER_ID);
    @Logged protected final TalonSwerveModule blModule = new TalonSwerveModule("BL", SwerveConstants.BL_DRIVE_ID, SwerveConstants.BL_TURN_ID, SwerveConstants.BL_ENCODER_ID);
    @Logged protected final TalonSwerveModule brModule = new TalonSwerveModule("BR", SwerveConstants.BR_DRIVE_ID, SwerveConstants.BR_TURN_ID, SwerveConstants.BR_ENCODER_ID);
    
    // - hardware
    // -- initializes the navX2 interface using the SPI channels of the MXP (myRIO Expansion Port) on the roboRIO
    // -- (the rectangular port in the center, below the NI or LabView logo)
    @Logged protected final AHRS navX2 = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // motion manipulation
    // - filters
    protected final SlewRateLimiter
        xSpeedLimiter = new SlewRateLimiter(SwerveConstants.TRANSLATIONAL_MAX_ACCELERATION.in(MetersPerSecondPerSecond)),
        ySpeedLimiter = new SlewRateLimiter(SwerveConstants.TRANSLATIONAL_MAX_ACCELERATION.in(MetersPerSecondPerSecond)),
        rotSpeedLimiter = new SlewRateLimiter(SwerveConstants.ROTATIONAL_MAX_ACCELERATION.in(RadiansPerSecondPerSecond));
    
    // - setpoint generation
    protected final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        SwerveConstants.FL_POS, SwerveConstants.FR_POS,
        SwerveConstants.BL_POS, SwerveConstants.BR_POS
    );

    // - odometry
    protected final SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(
        // the kinematics, so the estimator can calculate the swerve module states over time to better estimate the current position
        kinematics, 
        // the current gyro angle, so the estimator can use this value as an offset to convert from provided gyro angles to pose rotations
        navX2.getRotation2d(),
        // the current swerve module positions, so the estimator knows the initial state
        getPositions(),
        // the current pose, so the estimator knows where the robot is and what direction it's facing
        new Pose2d()
    );

    // data
    // - instance properties
    /** time between each call of robotPeriodic() */
    protected final double period;

    // - logging
    @Logged private SwerveModuleState[] lastStates;

    @Logged private double lastDifference;
    @Logged private double lastAverageDifference;

    // organization
    protected final TalonSwerveModule[] modules = { flModule, frModule, blModule, brModule };

    // initialization
    public Swerve(double period) {
        // set provided parameters
        this.period = period;

        // invert motors for rotation direction parity
        // TODO fix hacky inversion
        // flModule.invertDriveMotor();
        frModule.invertDriveMotor();
        // blModule.invertDriveMotor();
        brModule.invertDriveMotor();
    }

    // state
    /** whether the sum of the error from each modules' turn PID controller is less than 0.1, i.e., whether the turn setpoint has been essentially reached by all modules */
    public boolean finishedAligning() {
        double totalErrorRadians = 0.0;

        for (TalonSwerveModule module : modules) {
            totalErrorRadians += module.turnErrorRadians();
        }

        return totalErrorRadians < 0.1;
    }

    // TODO fix hacky actual state logging
    @Logged
    public SwerveModuleState[] getDepictedStates() {
        return new SwerveModuleState[]{
            flModule.getDepictedState(),
            frModule.getDepictedState(),
            blModule.getDepictedState(),
            brModule.getDepictedState()
        };
    }

    @Logged
    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[]{
            flModule.getPosition(),
            frModule.getPosition(),
            blModule.getPosition(),
            brModule.getPosition()
        };
    }

    // drive
    // TODO migrate this function to SwerveInputProcessor, as it deals with controller values, when this subsystem should only deal with velocities and positions
    /** drives the robot with the given robot-relative x controller speed, y controller speed, and controller rotational velocity */
    public void arcadeDrive(double xSpeed, double ySpeed, double rotSpeed) {
        // deadband x, y, and rotation speeds to avoid accidental idle drift
        xSpeed = MathUtil.applyDeadband(xSpeed, SwerveConstants.TRANSLATIONAL_SPEED_DEADBAND);
        ySpeed = MathUtil.applyDeadband(ySpeed, SwerveConstants.TRANSLATIONAL_SPEED_DEADBAND);
        rotSpeed = MathUtil.applyDeadband(rotSpeed, SwerveConstants.ROTATIONAL_SPEED_DEADBAND);

        // convert velocity values from the unitless range [-1, 1] to the range with units [-max speed, max speed]
        LinearVelocity xVel = SwerveConstants.TRANSLATIONAL_MAX_SPEED.times(xSpeed);
        LinearVelocity yVel = SwerveConstants.TRANSLATIONAL_MAX_SPEED.times(ySpeed);
        AngularVelocity angVel = SwerveConstants.ROTATIONAL_MAX_SPEED.times(rotSpeed);

        drive(xVel, yVel, angVel);
    }

    /** drives the robot with the given robot-relative x velocity, y velocity, and angular velocity */
    public void drive(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity angVel) {
        // limit maximum acceleration (change in speed over time) to avoid damaging the robot
        xVel = MetersPerSecond.of(xSpeedLimiter.calculate(xVel.in(MetersPerSecond)));
        yVel = MetersPerSecond.of(ySpeedLimiter.calculate(yVel.in(MetersPerSecond)));
        angVel = RadiansPerSecond.of(rotSpeedLimiter.calculate(angVel.in(RadiansPerSecond)));

        // create the chassis speeds class from our values (essentially just a struct containing them)
        ChassisSpeeds speeds = new ChassisSpeeds(xVel, yVel, angVel);

        // discretize the speeds to compensate for the error caused by a lack of continuous velocity modification
        // https://www.chiefdelphi.com/t/looking-for-an-explanation-of-chassisspeeds-discretize/462069/2
        speeds = ChassisSpeeds.discretize(speeds, period);

        // perform inverse kinematics using the provided SwerveDriveKinematics class to receive swerve module states in FL, FR, BL, BR order
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        
        // update swerve module setpoints
        // TODO fix hacky logging
        lastStates = states;

        for (int i = 0; i < 4; i++) {
            modules[i].updateSetpoint(states[i]);
        }

        // print range of speeds
        // average drive velocity
        double average = 0.0;

        for (TalonSwerveModule module : modules) {
            average += module.driveVelocity().in(MetersPerSecond);
        }

        average /= 4.0;

        // total difference from the average drive velocity
        double difference = 0.0;

        for (TalonSwerveModule module : modules) {
            difference += Math.abs(module.driveVelocity().in(MetersPerSecond) - average);
        }

        // average difference from the average drive velocity
        double averageDifference = difference / 4.0;

        lastDifference = difference;
        lastAverageDifference = averageDifference;
    }

    /** aligns the wheels to face a single direction, where zero is defined as the yaw the robot had when the navX was last zeroed */
    public void align(Angle position) {
        // set the setpoint's angle to be the same as the robot's direction, so all motors move forward
        SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(position));

        // update setpoints
        // TODO fix hacky logging
        lastStates = new SwerveModuleState[]{ state, state, state, state };

        for (int i = 0; i < 4; i++) {
            modules[i].updateSetpoint(state);
        }
    }

    /** controls a single swerve module using a controller speed value and angle, using the same alignment rules as align(Angle) */
    public void only(int moduleId, double speed, Angle position) {
        if (moduleId < 0 || moduleId > 4) {
            System.out.println("[Drive#only] Cannot control a single module with an id outside [0, 3]!");
            return;
        }

        // zero the voltage of all other modules
        for (int i = 0; i < 4; i++) {
            if (i == moduleId) continue;
            modules[i].zeroVoltage();
        }

        // convert controller speed to speed in m/s
        LinearVelocity vel = SwerveConstants.TRANSLATIONAL_MAX_SPEED.times(speed);

        // compose speed and angle into a swerve module state
        SwerveModuleState state = new SwerveModuleState(vel.in(MetersPerSecond), new Rotation2d(position));

        // update the setpoint
        // TODO fix this somewhat cursed hack for logging states
        lastStates = new SwerveModuleState[]{ new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
        lastStates[moduleId] = state;

        modules[moduleId].updateSetpoint(state);
    }

    /** controls a single swerve module using a controller speed value */
    public void only(int moduleId, double speed) {
        if (moduleId < 0 || moduleId > 4) {
            System.out.println("[Drive#only] Cannot control a single module with an id outside [0, 3]!");
            return;
        }

        // zero the voltage of all other modules
        for (int i = 0; i < 4; i++) {
            if (i == moduleId) continue;
            modules[i].zeroVoltage();
        }

        // convert controller speed to speed in m/s
        LinearVelocity vel = SwerveConstants.TRANSLATIONAL_MAX_SPEED.times(speed);

        // compose speed into a swerve module state
        SwerveModuleState state = new SwerveModuleState(vel.in(MetersPerSecond), null);

        // update the setpoint
        // TODO fix hacky logging
        lastStates = new SwerveModuleState[]{ new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
        lastStates[moduleId] = state;

        modules[moduleId].updateDriveSetpoint(state);
    }

    // actions
    /** performs the specified action for all swerve modules */
    public void perform(Consumer<TalonSwerveModule> action) {
        for (TalonSwerveModule module : modules) {
            action.accept(module);
        }
    }

    /** performs the specified action for the specified swerve module */
    public void perform(ModuleId id, Consumer<TalonSwerveModule> action) {
        action.accept(modules[id.index]);
    }

    // zeroing voltage
    /** zeroes the voltage of all modules */
    public void zeroVoltage() {
        // TODO fix hacky logging
        lastStates = new SwerveModuleState[]{ new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };

        perform(TalonSwerveModule::zeroVoltage);
    }

    /** zeroes the voltage of the specified module */
    public void zeroVoltage(ModuleId id) {
        // TODO fix hacky logging
        lastStates[id.index] = new SwerveModuleState();

        perform(id, TalonSwerveModule::zeroVoltage);
    }

    /** zeroes the drive voltage of all modules */
    public void zeroDriveVoltage() {
        // TODO fix hacky logging
        for (SwerveModuleState state : lastStates) {
            state.speedMetersPerSecond = 0.0;
        }

        perform(TalonSwerveModule::zeroDriveVoltage);
    }

    /** zeroes the drive voltage of the specified module */
    public void zeroDriveVoltage(ModuleId id) {
        // TODO fix hacky logging
        lastStates[id.index].speedMetersPerSecond = 0.0;

        perform(TalonSwerveModule::zeroDriveVoltage);
    }

    /** zeroes the turn voltage of all modules */
    public void zeroTurnVoltage() {
        // TODO fix hacky logging
        for (int i = 0; i < 4; i++) {
            lastStates[i].angle = new Rotation2d(modules[i].turnPosition());
        }

        perform(TalonSwerveModule::zeroTurnVoltage);
    }

    /** zeroes the turn voltage of the specified module */
    public void zeroTurnVoltage(ModuleId id) {
        // TODO fix hacky logging
        lastStates[id.index].angle = new Rotation2d(modules[id.index].turnPosition());
        
        perform(id, TalonSwerveModule::zeroTurnVoltage);
    }

    // classes
    public static enum ModuleId {
        FL(0), FR(1), BL(2), BR(3);

        /** the module's index in the <code>Swerve.modules</code> array */
        public final int index;

        private ModuleId(int index) {
            this.index = index;
        }
    }
}
