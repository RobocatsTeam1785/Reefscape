package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import java.util.Map;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.mode.ModeState;
import frc.robot.modes.DriveMode;
import frc.robot.subsystems.Drive;

public class RobotContainer {
    // subsystems
    private final Drive drive;

    // modes
    private ModeState<DriveMode> driveState = new ModeState<>(DriveMode.ALIGN);

    // controllers use NED CCC (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
    private final CommandXboxController
        driver = new CommandXboxController(0);

    public RobotContainer(double period) {
        drive = new Drive(period);

        configureCommands();
    }

    // commands
    private void configureCommands() {
        // mode-switching
        // d-pad up: a is swerve, and b is align
        driveState.registerSwitch(DriveMode.SWERVE, driver.povUp().and(driver.a()));
        driveState.registerSwitch(DriveMode.ALIGN, driver.povUp().and(driver.b()));
        
        // d-pad right: y is FL, b is FR, x is BL, a is BR, so normal directions but rotated 45 degrees CW
        driveState.registerSwitch(DriveMode.FL_ONLY, driver.povRight().and(driver.y()));
        driveState.registerSwitch(DriveMode.FR_ONLY, driver.povRight().and(driver.b()));
        driveState.registerSwitch(DriveMode.BL_ONLY, driver.povRight().and(driver.x()));
        driveState.registerSwitch(DriveMode.BR_ONLY, driver.povRight().and(driver.a()));

        // buttons
        // when a is pressed and in a single-module-only mode, zero the relative turn encoder
        // d-pad is negated to avoid collision with mode-switching
        driver.a().and(driveState.noSwitchesActive()).onTrue(new InstantCommand(() -> {
            DriveMode mode = driveState.mode();

            if (mode.oneModuleOnly()) {
                drive.zeroRelTurnEncoder(mode.id);
            }
        }));

        // state-based
        // zero turn voltage when the right trigger is lifted
        driver.rightTrigger(0.5).negate().onTrue(new InstantCommand(() -> {
            DriveMode mode = driveState.mode();

            if (mode.oneModuleOnly()) {
                drive.zeroTurnVoltage(mode.id);
            } else if (mode == DriveMode.ALIGN) {
                drive.zeroTurnVoltage();
            }
        }));

        // default commands
        drive.setDefaultCommand(driveState.selectRunnable(Map.of(
            DriveMode.SWERVE, this::driveSwerve,
            DriveMode.ALIGN, this::driveAlign,
            DriveMode.FL_ONLY, () -> driveOnly(0),
            DriveMode.FR_ONLY, () -> driveOnly(1),
            DriveMode.BL_ONLY, () -> driveOnly(2),
            DriveMode.BR_ONLY, () -> driveOnly(3)
        ), drive));
    }

    // periodic
    public void periodic() {
        // dashboard
        drive.updateDashboardValues();

        SmartDashboard.putString("drive mode", driveState.mode().toString());
    }

    // controller values
    /**
     * the rotation designated by the driver left joystick, where fully right means zero, and counterclockwise is positive
     * in turn, "zero" is defined as the yaw the robot had when the navX was last zeroed
     */
    public Angle leftAlignAngle() {
        // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC

        // fully right means 1, which is intuitive in ENU
        double x = driver.getLeftX();

        // fully up means -1, which is unintuitive in ENU, so it requires inversion
        double y = -driver.getLeftY();

        double angleRadians = Math.atan2(y, x);
        return Radians.of(angleRadians);
    }

    /**
     * the rotation designated by the driver right joystick, where fully right means zero, and counterclockwise is positive
     * in turn, "zero" is defined as the yaw the robot had when the navX was last zeroed
     */
    public Angle rightAlignAngle() {
        // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC

        // fully right means 1, which is intuitive in ENU
        double x = driver.getRightX();

        // fully up means -1, which is unintuitive in ENU, so it requires inversion
        double y = -driver.getRightY();

        double angleRadians = Math.atan2(y, x);
        return Radians.of(angleRadians);
    }

    // driving
    /** drives in swerve mode using the left joystick for translation and the right joystick for rotation */
    public void driveSwerve() {
        // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC
        // fully right means 1, which is intuitive
        double xSpeed = driver.getLeftX();

        // fully up means -1, which is unintuitive, so it requires inversion
        double ySpeed = -driver.getLeftY();

        // fully right means 1, which is positive; however, in WPILib, positive rotation means CCW rotation, and moving the joystick right is generally
        // associated with CW rotation, so it requires inversion
        double rotSpeed = -driver.getRightX();

        drive.arcadeDrive(xSpeed, ySpeed, rotSpeed);
    }

    /**
     * aligns to the rotation designated by the driver left joystick, where fully right means zero, and counterclockwise is positive
     * in turn, "zero" is defined as the yaw the robot had when the navX was last zeroed
     * 
     * alignment only occurs when the right trigger is down
     */
    public void driveAlign() {
        if (driver.getRightTriggerAxis() <= 0.5) return;

        drive.align(leftAlignAngle());
    }

    /**
     * drives a single swerve module using the left joystick (up and down meaning forward and backward) and the right joystick to control angle,
     * where the id is in [0, 3], in the order FL, FR, BL, BR
     * 
     * alignment only occurs when the right trigger is down
     */
    public void driveOnly(int moduleId) {
        boolean alignEnabled = driver.getRightTriggerAxis() > 0.5;

        // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC
        // fully up means -1, which is unintuitive in ENU, so it requires inversion
        double speed = -driver.getLeftY();

        if (alignEnabled) {
            drive.only(moduleId, speed, rightAlignAngle());
        } else {
            drive.only(moduleId, speed);
        }
    }
}
