package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.SwerveConstants;
import frc.lib.swerve.TalonSwerveModule;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Swerve extends SubsystemBase {
    // components
    // - abstracted
    @Logged public final TalonSwerveModule flModule = new TalonSwerveModule("FL", SwerveConstants.FL_DRIVE_ID, SwerveConstants.FL_TURN_ID, SwerveConstants.FL_ENCODER_ID);
    @Logged public final TalonSwerveModule frModule = new TalonSwerveModule("FR", SwerveConstants.FR_DRIVE_ID, SwerveConstants.FR_TURN_ID, SwerveConstants.FR_ENCODER_ID);
    @Logged public final TalonSwerveModule blModule = new TalonSwerveModule("BL", SwerveConstants.BL_DRIVE_ID, SwerveConstants.BL_TURN_ID, SwerveConstants.BL_ENCODER_ID);
    @Logged public final TalonSwerveModule brModule = new TalonSwerveModule("BR", SwerveConstants.BR_DRIVE_ID, SwerveConstants.BR_TURN_ID, SwerveConstants.BR_ENCODER_ID);
    
    // - hardware
    // -- initializes the navX2 interface using the SPI channels of the MXP (myRIO Expansion Port) on the roboRIO
    // -- (the rectangular port in the center, below the NI or LabView logo)
    @Logged public final AHRS navX2 = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // motion manipulation
    // - filters
    public final SlewRateLimiter
        xSpeedLimiter = new SlewRateLimiter(SwerveConstants.TRANSLATIONAL_MAX_ACCELERATION.in(MetersPerSecondPerSecond)),
        ySpeedLimiter = new SlewRateLimiter(SwerveConstants.TRANSLATIONAL_MAX_ACCELERATION.in(MetersPerSecondPerSecond)),
        rotSpeedLimiter = new SlewRateLimiter(SwerveConstants.ROTATIONAL_MAX_ACCELERATION.in(RadiansPerSecondPerSecond));
    
    // - setpoint generation
    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        SwerveConstants.FL_POS, SwerveConstants.FR_POS,
        SwerveConstants.BL_POS, SwerveConstants.BR_POS
    );

    // - odometry
    public final SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(
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
    public final double period;

    /** function that converts the previous estimated pose to an optional estimated global pose using vision input */
    public final Function<Pose2d, Optional<EstimatedRobotPose>> getEstimatedGlobalPose;

    // - logging
    @Logged public SwerveModuleState[] lastStates;

    @Logged public double lastDifference;
    @Logged public double lastAverageDifference;

    @Logged public Pose2d lastEstimatedVisionPose;

    // organization
    public final TalonSwerveModule[] modules = { flModule, frModule, blModule, brModule };

    // initialization
    public Swerve(double period, Function<Pose2d, Optional<EstimatedRobotPose>> getEstimatedGlobalPose) {
        // set provided parameters
        this.period = period;
        this.getEstimatedGlobalPose = getEstimatedGlobalPose;

        // invert motors for rotation direction parity
        // TODO fix hacky inversion
        // flModule.invertDriveMotor();
        // frModule.invertDriveMotor();
        // blModule.invertDriveMotor();
        // brModule.invertDriveMotor();

        // configure autos
        AutoBuilder.configure(
            estimator::getEstimatedPosition,
            estimator::resetPose,
            this::robotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(SwerveConstants.TRANSLATIONAL_KP, SwerveConstants.TRANSLATIONAL_KI, SwerveConstants.TRANSLATIONAL_KD), 
                new PIDConstants(SwerveConstants.ROTATIONAL_KP, SwerveConstants.ROTATIONAL_KI, SwerveConstants.ROTATIONAL_KD)
            ),
            SwerveConstants.PATHPLANNER_ROBOT_CONFIG,
            SwerveConstants.SHOULD_FLIP_PATH,
            this
        );
    }

    // periodic
    @Override
    public void periodic() {
        Pose2d previousPose = estimator.getEstimatedPosition();
        Optional<EstimatedRobotPose> maybeVisionPose = getEstimatedGlobalPose.apply(previousPose);

        estimator.update(navX2.getRotation2d(), getPositions());

        if (maybeVisionPose.isPresent()) {
            EstimatedRobotPose visionPose = maybeVisionPose.get();

            Pose2d visionRobotPoseMeters = visionPose.estimatedPose.toPose2d();
            double timestampSeconds = visionPose.timestampSeconds;

            lastEstimatedVisionPose = visionRobotPoseMeters;

            estimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
        }
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

    @Logged
    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[]{
            flModule.getState(),
            frModule.getState(),
            blModule.getState(),
            brModule.getState()
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

    @Logged
    public ChassisSpeeds robotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getStates());
    }

    // drive
    public void driveFieldRelative(LinearVelocity xVelocity, LinearVelocity yVelocity, AngularVelocity angularVelocity) {
        driveFieldRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        // TODO confirm robot angle is CCW+ and zeroed when facing frontwards
        double gyroAngle = -navX2.getAngle();
        Rotation2d robotAngle = Rotation2d.fromDegrees(gyroAngle);

        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle));
    }

    /** drives the robot with the given robot-relative x velocity, y velocity, and angular velocity */
    public void driveRobotRelative(LinearVelocity xVelocity, LinearVelocity yVelocity, AngularVelocity angularVelocity) {
        driveRobotRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
    }

    /** drives the robot with the given robot-relative ChassisSpeeds */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        // limit maximum acceleration (change in speed over time) to avoid damaging the robot
        speeds.vxMetersPerSecond = xSpeedLimiter.calculate(speeds.vxMetersPerSecond);
        speeds.vyMetersPerSecond = ySpeedLimiter.calculate(speeds.vyMetersPerSecond);
        speeds.omegaRadiansPerSecond = rotSpeedLimiter.calculate(speeds.omegaRadiansPerSecond);

        // discretize the speeds to compensate for the error caused by a lack of continuous velocity modification
        // https://www.chiefdelphi.com/t/looking-for-an-explanation-of-chassisspeeds-discretize/462069/2
        speeds = ChassisSpeeds.discretize(speeds, period);

        SmartDashboard.putNumber("s disc x speed m|s", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("s disc y speed m|s", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("s disc rot speed rad|s", speeds.omegaRadiansPerSecond);

        // perform inverse kinematics using the provided SwerveDriveKinematics class to receive swerve module states in FL, FR, BL, BR order
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        
        // update swerve module setpoints
        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber(i + " state vel m|s", states[i].speedMetersPerSecond);
            SmartDashboard.putNumber(i + " state turn pos rad", states[i].angle.getRadians());

            modules[i].updateSetpoint(states[i], i);
        }

        // update logging
        updateLogging(states);
    }

    /** aligns the wheels to face a single direction, where zero is defined as the yaw the robot had when the navX was last zeroed */
    public void align(Angle position) {
        // set the setpoint's angle to be the same as the robot's direction, so all motors move forward
        SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(position));

        // update setpoints
        // TODO fix hacky logging
        // lastStates = new SwerveModuleState[]{ state, state, state, state };

        for (int i = 0; i < 4; i++) {
            modules[i].updateSetpoint(state, i);
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
        // lastStates = new SwerveModuleState[]{ new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
        // lastStates[moduleId] = state;

        modules[moduleId].updateSetpoint(state, moduleId);
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
        SwerveModuleState state = new SwerveModuleState(vel.in(MetersPerSecond), new Rotation2d(modules[moduleId].turnPosition()));

        // update the setpoint
        // TODO fix hacky logging
        // lastStates = new SwerveModuleState[]{ new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
        // lastStates[moduleId] = state;

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
        // lastStates = new SwerveModuleState[]{ new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };

        perform(TalonSwerveModule::zeroVoltage);
    }

    /** zeroes the voltage of the specified module */
    public void zeroVoltage(ModuleId id) {
        // TODO fix hacky logging
        // lastStates[id.index] = new SwerveModuleState();

        perform(id, TalonSwerveModule::zeroVoltage);
    }

    /** zeroes the drive voltage of all modules */
    public void zeroDriveVoltage() {
        // TODO fix hacky logging
        // for (SwerveModuleState state : lastStates) {
        //     state.speedMetersPerSecond = 0.0;
        // }

        perform(TalonSwerveModule::zeroDriveVoltage);
    }

    /** zeroes the drive voltage of the specified module */
    public void zeroDriveVoltage(ModuleId id) {
        // TODO fix hacky logging
        // lastStates[id.index].speedMetersPerSecond = 0.0;

        perform(TalonSwerveModule::zeroDriveVoltage);
    }

    /** zeroes the turn voltage of all modules */
    public void zeroTurnVoltage() {
        // TODO fix hacky logging
        // for (int i = 0; i < 4; i++) {
        //     lastStates[i].angle = new Rotation2d(modules[i].turnPosition());
        // }

        perform(TalonSwerveModule::zeroTurnVoltage);
    }

    /** zeroes the turn voltage of the specified module */
    public void zeroTurnVoltage(ModuleId id) {
        // TODO fix hacky logging
        // lastStates[id.index].angle = new Rotation2d(modules[id.index].turnPosition());
        
        perform(id, TalonSwerveModule::zeroTurnVoltage);
    }

    // logging
    // TODO fix hacky logging
    public void updateLogging(SwerveModuleState[] states) {
        // calculate average drive velocity
        double average = 0.0;

        for (TalonSwerveModule module : modules) {
            average += module.driveVelocity().in(MetersPerSecond);
        }

        average /= 4.0;

        // calculate total difference from the average drive velocity
        double difference = 0.0;

        for (TalonSwerveModule module : modules) {
            difference += Math.abs(module.driveVelocity().in(MetersPerSecond) - average);
        }

        // calculate average difference from the average drive velocity
        double averageDifference = difference / 4.0;

        // record new values
        // lastStates = states;

        lastDifference = difference;
        lastAverageDifference = averageDifference;
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
