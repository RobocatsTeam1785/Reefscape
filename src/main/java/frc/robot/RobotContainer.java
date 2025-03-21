package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.constants.AutoConstants;
import frc.lib.constants.SwerveConstants;
import frc.lib.input.MasterInputProcessor;
import frc.robot.generated.TunerConstants;
import frc.robot.input.comp.CompInputProcessor;


import frc.robot.input.debug.DebugInputProcessor;
import frc.robot.input.shuffleboard.ShuffleboardInputProcessor;
import frc.robot.input.shuffleboard.TestInputProcessor;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralWheel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {
    // subsystems
    public Vision vision;

    // @Logged public Swerve swerve;
    public CommandSwerveDrivetrain swerve;
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

    public static int lastTagId;
    public static Pose3d lastTagPose;
    public static Rotation2d lastToTagAngle;

    // commands
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.times(SwerveConstants.TRANSLATIONAL_SPEED_DEADBAND))
            .withRotationalDeadband(SwerveConstants.ROBOT_ROTATIONAL_MAX_SPEED.times(SwerveConstants.ROTATIONAL_SPEED_DEADBAND)) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public RobotContainer(double period) {
        // subsystems
        vision = new Vision();

        // swerve = new Swerve(period, vision::getEstimatedGlobalPose);
        swerve = TunerConstants.createDrivetrain();
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
        // autos = new Autos();

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

        // add vision measurement and log relative X, Y, and angle and absolute angle
        Pose2d previousPose = swerve.getState().Pose;
        Optional<EstimatedRobotPose> maybeVisionPose = vision.getEstimatedGlobalPose(previousPose);

        if (maybeVisionPose.isPresent()) {
            EstimatedRobotPose visionPose = maybeVisionPose.get();

            Pose2d visionRobotPoseMeters = visionPose.estimatedPose.toPose2d();
            double timestampSeconds = visionPose.timestampSeconds;

            // swerve.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);

            // offset
            if (vision.latestResult != null && vision.latestResult.hasTargets()) {
                PhotonTrackedTarget tag = vision.latestResult.getBestTarget();
                Transform3d toTag = tag.bestCameraToTarget;

                int id = tag.getFiducialId();

                double relativeX = toTag.getX();
                double relativeY = toTag.getY();
                double relativeAngle = toTag.getRotation().getZ();

                SmartDashboard.putNumber("Tag ID", id);

                SmartDashboard.putNumber("Tag relative X", relativeX);
                SmartDashboard.putNumber("Tag relative Y", relativeY);
                SmartDashboard.putNumber("Tag relative angle", relativeAngle);

                lastToTagAngle = toTag.getRotation().toRotation2d();

                Optional<Pose3d> maybeTagPose = AutoConstants.layout.getTagPose(id);

                if (maybeTagPose.isPresent()) {
                    Pose3d tagPose = maybeTagPose.get();

                    double tagAngle = tagPose.getRotation().getZ();

                    SmartDashboard.putNumber("Tag angle", tagAngle);

                    // update state
                    lastTagPose = tagPose;
                }

                // update state
                lastTagId = id;
            }
        }
    }

    // rotation of the default driveRobotRelative to apply the same rotation the default drive controller left joystick control applies
    // public void drive(LinearVelocity oxVel, LinearVelocity oyVel, AngularVelocity angVel) {
    //     LinearVelocity xSpeed = oyVel;
    //     LinearVelocity ySpeed = oxVel.unaryMinus();

    //     swerve.driveRobotRelative(xSpeed, ySpeed, angVel);
    // }

    public void autonomousPeriodic() {
        if (movementTimer.isRunning()) {
            // move -1 m/s in the X direction, and since positive X is defined as forwards, that's moving 1 m/s backwards
            swerve.applyRequest(() ->
                drive.withVelocityX(MetersPerSecond.of(-1.0))
            ).schedule();

            if (movementTimer.hasElapsed(1.0)) {
                movementTimer.stop();

                // zero it to make it stop
                swerve.applyRequest(() ->
                    drive.withVelocityX(MetersPerSecond.of(0.0))
                ).schedule();
            }
        }
    }

    // init
    public void onDisabled() {
        // compProcessor.elevatorHeightMeters = 0.0;
    }

    public void autonomousInit() {
        // zero gyro based on roborio facing driver station
        swerve.seedFieldCentric();

        // update timers
        movementTimer.reset();
        movementTimer.start();
    }
}
