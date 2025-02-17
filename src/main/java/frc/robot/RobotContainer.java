package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.input.SwerveInputProcessor;
import frc.robot.subsystems.Drive;

public class RobotContainer {
    // subsystems
    private final Drive drive;

    // controllers use NED CCC (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
    private final CommandXboxController
        driver = new CommandXboxController(0);
    
    // input processors
    private final SwerveInputProcessor swerveProcessor;

    public RobotContainer(double period) {
        // subsystems
        drive = new Drive(period);

        // processors
        swerveProcessor = new SwerveInputProcessor(drive, driver);

        // processor configuration
        swerveProcessor.configureProcessing();
    }
}
