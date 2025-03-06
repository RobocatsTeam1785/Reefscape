package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.input.comp.CompInputProcessor;
import frc.robot.input.debug.DebugInputProcessor;
import frc.robot.input.shuffleboard.ShuffleboardInputProcessor;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeWheel;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralWheel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {
    // subsystems
    public Vision vision;

    @Logged public Swerve swerve;
    @Logged public Elevator elevator;

    @Logged public CoralArm coralArm;
    public CoralWheel coralWheel;

    public AlgaeArm algaeArm;
    public AlgaeWheel algaeWheel;

    // autos
    public Autos autos;

    // controllers use NED CCC (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);
    // public final CommandXboxController controller3 = new CommandXboxController(1);
    
    // input processors
    @Logged public DebugInputProcessor processor;
    @Logged public CompInputProcessor compProcessor;
    public ShuffleboardInputProcessor shuffleboardProcessor;

    public RobotContainer(double period) {
        // subsystems
        vision = new Vision();

        swerve = new Swerve(period, vision::getEstimatedGlobalPose);
        elevator = new Elevator();

        coralArm = new CoralArm();
        coralWheel = new CoralWheel();

        algaeArm = new AlgaeArm();
        algaeWheel = new AlgaeWheel();

        // processors
        // processor = new DebugInputProcessor(swerve, elevator, coralArm, coralWheel, algaeArm, algaeWheel, controller3);
        // processor.configure();

        compProcessor = new CompInputProcessor(swerve, elevator, coralArm, coralWheel, algaeArm, algaeWheel, driver, operator);
        compProcessor.configure();

        // shuffleboardProcessor = new ShuffleboardInputProcessor("Control", swerve, elevator, coralArm, coralWheel, algaeArm, algaeWheel);

        // autos
        autos = new Autos();
    }

    // periodic
    public void periodic() {
        // processor.periodic();
        // shuffleboardProcessor.periodic();
        compProcessor.periodic();

        elevator.tune();
        coralArm.tune();
        coralWheel.tune();
        algaeArm.tune();
        algaeWheel.tune();
    }

    // init
    public void onDisabled() {
        compProcessor.elevatorHeightMeters = 0.0;
    }
}
