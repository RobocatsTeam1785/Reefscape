package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.input.SwerveInputProcessor;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
    // subsystems
    private final Swerve swerve;

    // controllers use NED CCC (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
    private final CommandXboxController
        driver = new CommandXboxController(0);
    
    // input processors
    private final SwerveInputProcessor swerveProcessor;

    public RobotContainer(double period) {
        // subsystems
        swerve = new Swerve(period);

        // processors
        swerveProcessor = new SwerveInputProcessor(swerve, driver);

        // processor configuration
        swerveProcessor.configureProcessing();
    }
}
