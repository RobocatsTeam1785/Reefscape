package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.input.SubsystemInputProcessor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {
    // subsystems
    // @Logged private final Swerve swerve;
    // @Logged private final Elevator elevator;
    @Logged private final Vision vision;

    // controllers use NED CCC (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
    private final CommandXboxController
        driver = new CommandXboxController(0);
    
    // input processors
    // @Logged private final SubsystemInputProcessor processor;

    public RobotContainer(double period) {
        // subsystems
        // swerve = new Swerve(period);
        // elevator = new Elevator();
        vision = new Vision();

        // processors
        // processor = new SubsystemInputProcessor(swerve, elevator, driver);
        // processor.configure();
    }

    public void periodic() {
        vision.periodic();
    }
}
