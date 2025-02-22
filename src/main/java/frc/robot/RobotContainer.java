package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.input.SubsystemInputProcessor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
    // subsystems
    private final Swerve swerve;
    private final Elevator elevator;

    // controllers use NED CCC (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
    private final CommandXboxController
        driver = new CommandXboxController(0);
    
    // input processors
    private final SubsystemInputProcessor processor;

    public RobotContainer(double period) {
        // subsystems
        swerve = new Swerve(period);
        elevator = new Elevator();

        // processors
        processor = new SubsystemInputProcessor(swerve, elevator, driver);
        processor.configure();
    }
}
