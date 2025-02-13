package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.utility.CommandUtils;
import frc.robot.subsystems.Drive;

public class RobotContainer {
    // subsystems
    private final Drive drive;

    // controllers use NED CCC (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
    private final CommandXboxController
        driver = new CommandXboxController(0),
        operator = new CommandXboxController(1);

    public RobotContainer(double period) {
        drive = new Drive(period);

        configureCommands();
    }

    private void configureCommands() {
        // buttons - currently empty

        // align to the rotation designated by the operator left joystick, where fully right means zero, and counterclockwise is positive
        // in turn, "zero" is defined as the yaw the robot had when the navX was last zeroed
        operator.a().onTrue(new InstantCommand(() -> {
            // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC
            System.out.println("Running align command");

            // fully right means 1, which is intuitive in ENU
            double x = operator.getLeftX();

            // fully up means -1, which is unintuitive in ENU, so it requires inversion
            double y = -operator.getLeftY();

            double angleRadians = Math.atan2(y, x);
            Angle angle = Radians.of(angleRadians);

            drive.align(angle);
        }, drive));

        // default commands
        CommandUtils.setDefault(drive, () -> {
            // arcade drive
            // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC
            System.out.println("Running drive command");
            
            // fully right means 1, which is intuitive
            double xSpeed = driver.getLeftX();

            // fully up means -1, which is unintuitive, so it requires inversion
            double ySpeed = -driver.getLeftY();

            // fully right means 1, which is positive; however, in WPILib, positive rotation means CCW rotation, and moving the joystick right is generally
            // associated with CW rotation, so it requires inversion
            double rotSpeed = -driver.getRightX();

            drive.arcadeDrive(xSpeed, ySpeed, rotSpeed);            
        });
    }
}
