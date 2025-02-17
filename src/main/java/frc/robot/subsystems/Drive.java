package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
    // filters and kinematics
    protected final SlewRateLimiter
        xSpeedLimiter = new SlewRateLimiter(Constants.Drive.TRANSLATIONAL_MAX_ACCELERATION.in(MetersPerSecondPerSecond)),
        ySpeedLimiter = new SlewRateLimiter(Constants.Drive.TRANSLATIONAL_MAX_ACCELERATION.in(MetersPerSecondPerSecond)),
        rotSpeedLimiter = new SlewRateLimiter(Constants.Drive.ROTATIONAL_MAX_ACCELERATION.in(RadiansPerSecondPerSecond));
    
    protected final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        Constants.Drive.FL_POS, Constants.Drive.FR_POS,
        Constants.Drive.BL_POS, Constants.Drive.BR_POS
    );

    // hardware
    protected final SwerveModule[] modules;

    // initializes the navX2 interface using the SPI channels of the MXP (myRIO Expansion Port) on the roboRIO
    // (the rectangular port in the center, below the NI or LabView logo)
    protected final AHRS navX2 = new AHRS(AHRS.NavXComType.kMXP_SPI);

    /** time between each call of robotPeriodic() */
    protected final double period;

    public Drive(double period) {
        this.period = period;

        modules = new SwerveModule[]{
            new SwerveModule("FL", Constants.Drive.FL_DRIVE_ID, Constants.Drive.FL_TURN_ID, Constants.Drive.FL_ENCODER_ID),
            new SwerveModule("FR", Constants.Drive.FR_DRIVE_ID, Constants.Drive.FR_TURN_ID, Constants.Drive.FR_ENCODER_ID),
            new SwerveModule("BL", Constants.Drive.BL_DRIVE_ID, Constants.Drive.BL_TURN_ID, Constants.Drive.BL_ENCODER_ID),
            new SwerveModule("BR", Constants.Drive.BR_DRIVE_ID, Constants.Drive.BR_TURN_ID, Constants.Drive.BR_ENCODER_ID)
        };
    }

    // state
    /** whether the sum of the error from each modules' turn PID controller is less than 0.1, i.e., whether the turn setpoint has been essentially reached by all modules */
    public boolean finishedAligning() {
        double totalErrorRadians = 0.0;

        for (int i = 0; i < 4; i++) {
            totalErrorRadians += modules[i].turnErrorRadians();
        }

        return totalErrorRadians < 0.1;
    }

    // dashboard
    /** sets the dashboard values for all swerve modules */
    public void updateDashboardValues() {
        for (SwerveModule module : modules) {
            module.updateDashboardValues();
        }
    }

    /** sets the dashboard values for the swerve module at the specified id */
    public void updateDashboardValues(int id) {
        if (id < 0 || id > 3) {
            System.out.println("[Drive#setDashboardValues(int)] cannot set the dashboard values for a swerve module with an id outside of bounds! expected an id in [0, 3], but found " + id + "!");
            return;
        }

        modules[id].updateDashboardValues();
    }

    // drive
    /** drives the robot with the given robot-relative x controller speed, y controller speed, and controller rotational velocity */
    public void arcadeDrive(double xSpeed, double ySpeed, double rotSpeed) {
        // deadband x, y, and rotation speeds
        // TODO figure out why andrew implemented a deadband, to see if I can remove it
        xSpeed = MathUtil.applyDeadband(xSpeed, Constants.Drive.TRANSLATIONAL_SPEED_DEADBAND);
        ySpeed = MathUtil.applyDeadband(ySpeed, Constants.Drive.TRANSLATIONAL_SPEED_DEADBAND);
        rotSpeed = MathUtil.applyDeadband(rotSpeed, Constants.Drive.ROTATIONAL_SPEED_DEADBAND);

        // convert velocity values from the unitless range [-1, 1] to the range with units [-max speed, max speed]
        LinearVelocity xVel = Constants.Drive.TRANSLATIONAL_MAX_SPEED.times(xSpeed);
        LinearVelocity yVel = Constants.Drive.TRANSLATIONAL_MAX_SPEED.times(ySpeed);
        AngularVelocity angVel = Constants.Drive.ROTATIONAL_MAX_SPEED.times(rotSpeed);

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
        for (int i = 0; i < 4; i++) {
            modules[i].updateSetpoint(states[i]);
        }
    }

    /** aligns the wheels to face a single direction, where zero is defined as the yaw the robot had when the navX was last zeroed */
    public void align(Angle position) {
        // set the setpoint's angle to be the same as the robot's direction, so all motors move forward
        SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(position));

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
        LinearVelocity vel = Constants.Drive.TRANSLATIONAL_MAX_SPEED.times(speed);

        // compose speed and angle into a swerve module state
        SwerveModuleState state = new SwerveModuleState(vel.in(MetersPerSecond), new Rotation2d(position));

        // update the setpoint
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
        LinearVelocity vel = Constants.Drive.TRANSLATIONAL_MAX_SPEED.times(speed);

        // compose speed into a swerve module state
        SwerveModuleState state = new SwerveModuleState(vel.in(MetersPerSecond), null);

        // update the setpoint
        modules[moduleId].updateDriveSetpoint(state);
    }

    // zeroing encoders
    /** zeroes the relative turn encoder of the specified module */
    public void zeroRelTurnEncoder(int moduleId) {
        if (moduleId < 0 || moduleId > 4) {
            System.out.println("[Drive#zeroRelTurnEncoder] Cannot zero a single module with an id outside [0, 3]!");
            return;
        }

        modules[moduleId].zeroRelTurnEncoder();
    }

    // zeroing voltage
    /** zeroes the voltage of all modules */
    public void zeroVoltage() {
        for (SwerveModule module : modules) {
            module.zeroVoltage();
        }
    }

    /** zeroes the voltage of the specified module */
    public void zeroVoltage(int moduleId) {
        if (moduleId < 0 || moduleId > 4) {
            System.out.println("[Drive#zeroVoltage] Cannot zero the voltage of a single module with an id outside [0, 3]!");
            return;
        }

        modules[moduleId].zeroVoltage();
    }

    /** zeroes the drive voltage of all modules */
    public void zeroDriveVoltage() {
        for (SwerveModule module : modules) {
            module.zeroDriveVoltage();
        }
    }

    /** zeroes the drive voltage of the specified module */
    public void zeroDriveVoltage(int moduleId) {
        if (moduleId < 0 || moduleId > 4) {
            System.out.println("[Drive#zeroDriveVoltage] Cannot zero the voltage of a single module with an id outside [0, 3]!");
            return;
        }

        modules[moduleId].zeroDriveVoltage();
    }

    /** zeroes the turn voltage of all modules */
    public void zeroTurnVoltage() {
        for (SwerveModule module : modules) {
            module.zeroTurnVoltage();
        }
    }

    /** zeroes the turn voltage of the specified module */
    public void zeroTurnVoltage(int moduleId) {
        if (moduleId < 0 || moduleId > 4) {
            System.out.println("[Drive#zeroTurnVoltage] Cannot zero the voltage of a single module with an id outside [0, 3]!");
            return;
        }

        modules[moduleId].zeroTurnVoltage();
    }
}
