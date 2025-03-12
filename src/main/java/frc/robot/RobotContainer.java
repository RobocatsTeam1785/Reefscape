package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.input.MasterInputProcessor;
import frc.robot.input.comp.CompInputProcessor;


import frc.robot.input.debug.DebugInputProcessor;
import frc.robot.input.shuffleboard.ShuffleboardInputProcessor;
import frc.robot.input.shuffleboard.TestInputProcessor;
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

    // public AlgaeArm algaeArm;
    // public AlgaeWheel algaeWheel;

    // autos
    public Autos autos;
    
    // input processors
    public final MasterInputProcessor[] processors;

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

        // processors
        processors = new MasterInputProcessor[]{
            // new DebugInputProcessor(swerve, elevator, coralArm, coralWheel, algaeArm, algaeWheel, controller3),
            new CompInputProcessor(swerve, elevator, coralArm, coralWheel, /* algaeArm, algaeWheel, */ 0, 1),
            // new ShuffleboardInputProcessor("Control", swerve, elevator, coralArm, coralWheel, algaeArm, algaeWheel),
            // new TestInputProcessor(3, swerve)
        };

        for (MasterInputProcessor processor : processors) {
            processor.configure();
        }

        // autos
        autos = new Autos();

        // timers
        movementTimer = new Timer();
        alignTimer = new Timer();
    }

    // periodic
    public void periodic() {
        for (MasterInputProcessor processor : processors) {
            processor.periodic();
        }

        elevator.tune();
        coralArm.tune();
        coralWheel.tune();
    }

    // rotation of the default driveRobotRelative to apply the same rotation the default drive controller left joystick control applies
    public void drive(LinearVelocity oxVel, LinearVelocity oyVel, AngularVelocity angVel) {
        LinearVelocity xSpeed = oyVel;
        LinearVelocity ySpeed = oxVel.unaryMinus();

        swerve.driveRobotRelative(xSpeed, ySpeed, angVel);
    }

    public void autonomousPeriodic() {
        if (alignTimer.isRunning()) {
            swerve.align(Degrees.of(90.0));

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
        // compProcessor.elevatorHeightMeters = 0.0;
    }

    public void autonomousInit() {
        // zero gyro based on roborio facing driver station
        swerve.navX2.zeroYaw();
        swerve.navX2.setAngleAdjustment(-(swerve.navX2.getAngle()) - 90.0);

        // update timers
        alignTimer.reset();
        alignTimer.start();
    }
}
