package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.utility.CommandUtils;
import frc.robot.modes.DriveMode;
import frc.robot.subsystems.Drive;

public class RobotContainer {
    // subsystems
    private final Drive drive;

    // modes
    private DriveMode mode = DriveMode.ALIGN;

    // controllers use NED CCC (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
    private final CommandXboxController
        driver = new CommandXboxController(0);

    public RobotContainer(double period) {
        drive = new Drive(period);

        configureCommands();
    }

    // commands
    private void configureCommands() {
        // buttons
        // d-pad up: a is swerve, and b is align
        driver.povUp().and(driver.a()).onTrue(new InstantCommand(() -> {
            mode = DriveMode.SWERVE;
        }));

        driver.povUp().and(driver.b()).onTrue(new InstantCommand(() -> {
            mode = DriveMode.ALIGN;
        }));

        // d-pad right: y is FL, b is FR, x is BL, a is BR, so normal directions but rotated 45 degrees CW
        driver.povRight().and(driver.y()).onTrue(new InstantCommand(() -> {
            mode = DriveMode.FL_ONLY;
        }));

        driver.povRight().and(driver.b()).onTrue(new InstantCommand(() -> {
            mode = DriveMode.FR_ONLY;
        }));

        driver.povRight().and(driver.x()).onTrue(new InstantCommand(() -> {
            mode = DriveMode.BL_ONLY;
        }));

        driver.povRight().and(driver.a()).onTrue(new InstantCommand(() -> {
            mode = DriveMode.BR_ONLY;
        }));

        // when a is pressed and in a single-module-only mode, zero the relative turn encoder
        // d-pad is negated to avoid collision with mode-switching
        driver.a().and(driver.povUp().negate()).and(driver.povRight().negate()).onTrue(new InstantCommand(() -> {
            if (mode.oneModuleOnly()) {
                drive.zeroRelTurnEncoder(mode.id);
            }
        }));

        // zero turn voltage when the right trigger is lifted
        driver.rightTrigger(0.5).negate().onTrue(new InstantCommand(() -> {
            if (mode.oneModuleOnly()) {
                drive.zeroTurnVoltage(mode.id);
            } else if (mode == DriveMode.ALIGN) {
                drive.zeroTurnVoltage();
            }
        }));

        // default commands
        CommandUtils.setDefault(drive, () -> {
            // only align when the right trigger is held down
            boolean alignEnabled = driver.getRightTriggerAxis() > 0.5;

            switch(mode) {
                case SWERVE -> driveSwerve();
                case ALIGN -> driveAlign(alignEnabled);
                case FL_ONLY -> driveOnly(0, alignEnabled);
                case FR_ONLY -> driveOnly(1, alignEnabled);
                case BL_ONLY -> driveOnly(2, alignEnabled);
                case BR_ONLY -> driveOnly(3, alignEnabled);
            }
        });
    }

    // periodic
    public void periodic() {
        // dashboard
        drive.updateDashboardValues();

        SmartDashboard.putString("drive mode", mode.toString());
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
     */
    public void driveAlign(boolean alignEnabled) {
        if (alignEnabled) {
            drive.align(leftAlignAngle());
        }
    }

    /**
     * drives a single swerve module using the left joystick (up and down meaning forward and backward) and the right joystick to control angle,
     * where the id is in [0, 3], in the order FL, FR, BL, BR
     */
    public void driveOnly(int moduleId, boolean alignEnabled) {
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
