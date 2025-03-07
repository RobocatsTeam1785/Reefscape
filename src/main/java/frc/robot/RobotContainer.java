package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
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
    // @Logged public DebugInputProcessor processor;
    @Logged public CompInputProcessor compProcessor;
    // public ShuffleboardInputProcessor shuffleboardProcessor;

    // data
    private Timer alignTimer;
    private Timer movementTimer;

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

        compProcessor.configureDriverTriggers();
        compProcessor.configureOperatorTriggers();

        compProcessor.configureDefaults();

        // shuffleboardProcessor = new ShuffleboardInputProcessor("Control", swerve, elevator, coralArm, coralWheel, algaeArm, algaeWheel);

        // autos
        autos = new Autos();

        // timers
        movementTimer = new Timer();
        alignTimer = new Timer();
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

    // rotation of the default driveRobotRelative to apply the same rotation the default drive controller left joystick control applies
    public void drive(LinearVelocity oxVel, LinearVelocity oyVel, AngularVelocity angVel) {
        LinearVelocity xSpeed = oyVel;
        LinearVelocity ySpeed = oxVel.unaryMinus();

        swerve.driveRobotRelative(xSpeed, ySpeed, angVel);
    }

    public void autonomousPeriodic() {
        if (alignTimer.isRunning()) {
            LinearVelocity xSpeed = MetersPerSecond.of(-0.1);
            LinearVelocity ySpeed = MetersPerSecond.of(0.0);
            AngularVelocity angularVelocity = RadiansPerSecond.of(0.0);

            drive(xSpeed, ySpeed, angularVelocity);

            if (alignTimer.hasElapsed(1.0)) {
                alignTimer.stop();

                movementTimer.reset();
                movementTimer.start();
            }
        } else if (movementTimer.isRunning()) {
            // zero movement
            LinearVelocity xSpeed = MetersPerSecond.of(-1.0);
            LinearVelocity ySpeed = MetersPerSecond.of(0.0);
            AngularVelocity angularVelocity = RadiansPerSecond.of(0.0);

            drive(xSpeed, ySpeed, angularVelocity);

            if (movementTimer.hasElapsed(1.0)) {
                movementTimer.stop();
            }
        } else {
            // zero movement
            LinearVelocity xSpeed = MetersPerSecond.of(0.0);
            LinearVelocity ySpeed = MetersPerSecond.of(0.0);
            AngularVelocity angularVelocity = RadiansPerSecond.of(0.0);

            drive(xSpeed, ySpeed, angularVelocity);
        }
    }

    // init
    public void onDisabled() {
        compProcessor.elevatorHeightMeters = 0.0;
    }

    public void autonomousInit() {
        // update timers
        alignTimer.reset();
        alignTimer.start();

        // move backwards
        LinearVelocity xSpeed = MetersPerSecond.of(-0.1);
        LinearVelocity ySpeed = MetersPerSecond.of(0.0);
        AngularVelocity angularVelocity = RadiansPerSecond.of(0.0);

        drive(xSpeed, ySpeed, angularVelocity);
    }
}
