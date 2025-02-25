package frc.robot;

import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.input.DebugInputProcessor;
// import frc.robot.subsystems.AlgaeArm;
// import frc.robot.subsystems.AlgaeWheel;
// import frc.robot.subsystems.CoralArm;
// import frc.robot.subsystems.CoralWheel;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {
    // subsystems
    // @Logged private Swerve swerve;
    // @Logged private Elevator elevator;

    // @Logged private CoralArm coralArm;
    // @Logged private CoralWheel coralWheel;

    // @Logged private AlgaeArm algaeArm;
    // @Logged private AlgaeWheel algaeWheel;

    @Logged private Vision vision;

    // autos
    private Autos autos;

    // controllers use NED CCC (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
    // private final CommandXboxController driver = new CommandXboxController(0);
    
    // input processors
    @Logged private DebugInputProcessor processor;

    public RobotContainer(double period) {
        // subsystems
        // swerve = new Swerve(period);
        // elevator = new Elevator();

        // coralArm = new CoralArm();
        // coralWheel = new CoralWheel();

        // algaeArm = new AlgaeArm();
        // algaeWheel = new AlgaeWheel();

        vision = new Vision();

        // processors
        // processor = new SubsystemInputProcessor(swerve, elevator, coralArm, coralWheel, algaeArm, algaeWheel, driver);
        // processor.configure();

        // autos
        autos = new Autos();
    }
}
