package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.constants.AutoConstants;
import frc.lib.constants.CoralArmConstants;
import frc.lib.constants.SwerveConstants;
import frc.lib.input.MasterInputProcessor;
import frc.robot.generated.TunerConstants;
import frc.robot.input.comp.CompInputProcessor;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralWheel;
import frc.robot.subsystems.Elevator;
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

    public Climber climber;

    // autos
    public Autos autos;
    
    // input processors
    public final MasterInputProcessor[] processors;

    // data
    // private Timer alignTimer;
    // private Timer movementTimer;

    public static int lastTagId;
    public static Pose3d lastTagPose;
    public static Rotation2d lastToTagAngle;

    // commands
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.times(SwerveConstants.TRANSLATIONAL_SPEED_DEADBAND))
            .withRotationalDeadband(SwerveConstants.ROBOT_ROTATIONAL_MAX_SPEED.times(SwerveConstants.ROTATIONAL_SPEED_DEADBAND)) // Add a 10% deadband
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    
    private final SwerveRequest.ApplyRobotSpeeds autoDrive = new SwerveRequest.ApplyRobotSpeeds();

    private final SendableChooser<List<Triple>> chooser;

    public RobotContainer(double period) {
        // subsystems
        vision = new Vision();

        // swerve = new Swerve(period, vision::getEstimatedGlobalPose);
        swerve = TunerConstants.createDrivetrain();
        elevator = new Elevator();

        // configure autos
        AutoBuilder.configure(
            () -> swerve.getState().Pose,
            swerve::resetPose,
            () -> swerve.getState().Speeds,
            (speeds, feedforwards) -> swerve.applyRequest(() -> {
                return autoDrive.withSpeeds(speeds);
            }),
            new PPHolonomicDriveController(
                new PIDConstants(SwerveConstants.TRANSLATIONAL_KP, SwerveConstants.TRANSLATIONAL_KI, SwerveConstants.TRANSLATIONAL_KD), 
                new PIDConstants(SwerveConstants.ROTATIONAL_KP, SwerveConstants.ROTATIONAL_KI, SwerveConstants.ROTATIONAL_KD)
            ),
            SwerveConstants.PATHPLANNER_ROBOT_CONFIG,
            SwerveConstants.SHOULD_FLIP_PATH,
            swerve
        );

        coralArm = new CoralArm();
        coralWheel = new CoralWheel();

        climber = new Climber();

        // processors
        processors = new MasterInputProcessor[]{
            // new DebugInputProcessor(swerve, elevator, coralArm, coralWheel, algaeArm, algaeWheel, controller3),
            new CompInputProcessor(swerve, elevator, coralArm, coralWheel, /* algaeArm, algaeWheel, */ climber, 0, 1),
            // new ShuffleboardInputProcessor("Control", swerve, elevator, coralArm, coralWheel, algaeArm, algaeWheel),
            // new TestInputProcessor(3, swerve)
        };

        for (MasterInputProcessor processor : processors) {
            processor.configure();
        }

        // autos
        autos = new Autos();

        // timers
        // movementTimer = new Timer();
        // alignTimer = new Timer();

        // auto shuffleboard stuff
        chooser = new SendableChooser<>();
        chooser.setDefaultOption("straight", straightL1Auto);
        chooser.addOption("long horizontal L1", longHorizontalL1Auto);
        chooser.addOption("long horizontal L2", longHorizontalL2Auto);
        chooser.addOption("move", onlyMoveAuto);

        SmartDashboard.putData("Custom auto chooser", chooser);
    }

    // periodic
    public void robotPeriodic() {
        // add vision measurement and log relative X, Y, and angle and absolute angle
        Pose2d previousPose = swerve.getState().Pose;
        Optional<EstimatedRobotPose> maybeVisionPose = vision.getEstimatedGlobalPose(previousPose);

        if (maybeVisionPose.isPresent()) {
            EstimatedRobotPose visionPose = maybeVisionPose.get();

            Pose2d visionRobotPoseMeters = visionPose.estimatedPose.toPose2d();
            double timestampSeconds = visionPose.timestampSeconds;

            swerve.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);

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

    public void periodic() {
        for (MasterInputProcessor processor : processors) {
            processor.periodic();
        }

        elevator.tune();
        coralArm.tune();
        coralWheel.tune();
    }

    // rotation of the default driveRobotRelative to apply the same rotation the default drive controller left joystick control applies
    // public void drive(LinearVelocity oxVel, LinearVelocity oyVel, AngularVelocity angVel) {
    //     LinearVelocity xSpeed = oyVel;
    //     LinearVelocity ySpeed = oxVel.unaryMinus();

    //     swerve.driveRobotRelative(xSpeed, ySpeed, angVel);
    // }

    // sequential autonomous commands
    // ! this is the section for customizing autos - the red is irrelevant, it's just to draw attention to this line
    private List<Triple> scoreL1Auto = List.of(
        new Triple(
            1.0,
            () -> flickCoralArmUp(1.0),
            () -> {}
        ),
        new Triple(
            1.0,
            () -> fallToAngle(Degrees.of(-33), 0.6),
            () -> {}
        ),
        new Triple(
            1.0,
            () -> spinCoralWheels(-3.0),
            () -> spinCoralWheels(0.0)
        )
    );

    private List<Triple> scoreL2Auto = List.of(
        new Triple(
            1.0,
            () -> flickCoralArmUp(1.0),
            () -> {}
        ),
        new Triple(
            1.0,
            () -> fallToAngle(Degrees.of(0), 0.4),
            () -> {}
        ),
        new Triple(
            1.0,
            () -> spinCoralWheels(-3.0),
            () -> spinCoralWheels(0.0)
        )
    );

    private List<Triple> straightL1Auto = List.of(
        new Triple(
            2.5,
            () -> drive(-1.0, 0.0, 0.0),
            () -> drive(0.0, 0.0, 0.0)
        )
    );

    private List<Triple> longHorizontalL1Auto = List.of(
        new Triple(
            2.5 * 1.33,
            () -> drive(-1.0, 0.0, 0.0),
            () -> drive(0.0, 0.0, 0.0)
        )
    );

    private List<Triple> longHorizontalL2Auto = List.of(
        new Triple(
            1.0,
            () -> flickCoralArmUp(1.0),
            () -> {}
        ),
        new Triple(
            1.0,
            () -> fallToAngle(Degrees.of(-33), 0.6),
            () -> {}
        ),

        new Triple(
            0.2,
            () -> drive(-0.001, 0.0, 0.0),
            () -> drive(0.0, 0.0, 0.0)
        ),

        new Triple(
            2.5 * 1.33 + 0.5,
            () -> drive(-1.0, 0.0, 0.0),
            () -> drive(0.0, 0.0, 0.0)
        ),

        new Triple(
            1.0,
            () -> spinCoralWheels(-3.0),
            () -> spinCoralWheels(0.0)
        ),

        new Triple(
            0.1,
            () -> {},
            () -> {}
        ),

        new Triple(
            1.0,
            () -> {
                spinCoralWheels(-1.0);
                drive(1.0, 0.0, 0.0);
            },
            () -> {
                spinCoralWheels(0.0);
                drive(0.0, 0.0, 0.0);
            }
        )
    );

    {
        straightL1Auto = new ArrayList<>(straightL1Auto);
        longHorizontalL1Auto = new ArrayList<>(longHorizontalL1Auto);
        longHorizontalL2Auto = new ArrayList<>(longHorizontalL2Auto);

        straightL1Auto.addAll(scoreL1Auto);
        longHorizontalL1Auto.addAll(scoreL1Auto);
    }

    private final List<Triple> onlyMoveAuto = List.of(
        new Triple(
            1.0,
            () -> drive(-1.0, 0.0, 0.0),
            () -> drive(0.0, 0.0, 0.0)
        )
    );

    private List<Triple> autoCommands = straightL1Auto;

    private int autoCommandIndex = 0;

    private final Timer autoCommandTimer = new Timer();

    private final TrapezoidProfile.Constraints robotTranslationalConstraints = new TrapezoidProfile.Constraints(5.0, 1.0);

    private final ProfiledPIDController xController = new ProfiledPIDController(
        SwerveConstants.TRANSLATIONAL_KP, SwerveConstants.TRANSLATIONAL_KI, SwerveConstants.TRANSLATIONAL_KD,
        robotTranslationalConstraints);
    
    private final ProfiledPIDController yController = new ProfiledPIDController(
        SwerveConstants.TRANSLATIONAL_KP, SwerveConstants.TRANSLATIONAL_KI, SwerveConstants.TRANSLATIONAL_KD,
        robotTranslationalConstraints);

    private final ProfiledPIDController omegaController = new ProfiledPIDController(
        SwerveConstants.ROTATIONAL_KP, SwerveConstants.ROTATIONAL_KI, SwerveConstants.ROTATIONAL_KD,
        SwerveConstants.ROBOT_TURN_PROFILE_CONSTRAINTS_RADIANS);

    // driver auto
    public void goToPose(Pose3d pose) {
        double xo = xController.calculate(swerve.getState().Pose.getX(), pose.getX());
        double yo = yController.calculate(swerve.getState().Pose.getY(), pose.getY());
        double ro = omegaController.calculate(swerve.getState().Pose.getRotation().getRadians(), pose.getRotation().toRotation2d().getRadians());

        swerve.applyRequest(() ->
            drive
                .withVelocityX(MetersPerSecond.of(xo))
                .withVelocityY(MetersPerSecond.of(yo))
                .withRotationalRate(RadiansPerSecond.of(ro))
        ).schedule();
    }

    public void drive(double x, double y, double omega) {
        swerve.applyRequest(() ->
            drive
                .withVelocityX(MetersPerSecond.of(x))
                .withVelocityY(MetersPerSecond.of(y))
                .withRotationalRate(RadiansPerSecond.of(omega))
        ).schedule();
    }

    public void stopDriving() {
        swerve.applyRequest(() -> drive).schedule();
    }

    /** rotates to the specified angle where facing the blue alliance is zero and CCW+ */
    public void rotateTo(Angle angle) {
        swerve.applyRequest(() ->
            driveFacing.withTargetDirection(new Rotation2d(angle))
        ).schedule();
    }

    // coral auto
    public void flickCoralArmUp(double volts) {
        coralArm.updateVoltage(Volts.of(volts));
    }

    public void flickCoralArmUp(double volts, Angle slowdownAngle, double slowerVolts) {
        if (coralArm.hexPosition().gt(slowdownAngle)) {
            coralArm.updateVoltage(Volts.of(slowerVolts));
        } else {
            coralArm.updateVoltage(Volts.of(volts));
        }
    }

    public void fallToAngle(Angle angle, double sustainingVolts) {
        if (coralArm.hexPosition().lt(angle)) {
            coralArm.updateVoltage(Volts.of(sustainingVolts));
        } else {
            coralArm.updateVoltage(Volts.zero());
        }
    }

    // coral wheel auto
    public void spinCoralWheels(double volts) {
        coralWheel.updateVoltage(Volts.of(volts));
    }

    // elevator auto
    /**
     * ! use EXTREME care with this method - because this is in auto, and thus uncontrollable, this has a very real risk of breaking the elevator if used improperly
     * ! additionally, the stopThresholdHeight should ALWAYS be safely below the actual max height to avoid damaging the elevator by going past the max height
     * 
     * ! also, you MUST make sure the motors are not inversed, as that will lead to the pulleys increasing the elevator's height while recording negative encoder values,
     * ! bypassing the slow and stop thresholds, which WILL break the elevator if not stopped in time
     */
    public void raiseElevator(double volts, Distance slowerHeight, double slowerVolts, Distance stopThresholdHeight, double holdVoltage) {
        if (elevator.leftHeight().gt(stopThresholdHeight)) {
            elevator.updateVoltage(Volts.of(holdVoltage));
        } else if (elevator.leftHeight().gt(slowerHeight)) {
            elevator.updateVoltage(Volts.of(slowerVolts));
        } else {
            elevator.updateVoltage(Volts.of(volts));
        }
    }

    public void ppAutoInit() {
        autos.autonomousCommand().schedule();
        System.out.println(autos.autonomousCommand());
    }

    public void autonomousPeriodic() {
        // if (movementTimer.isRunning()) {
        //     // move -1 m/s in the X direction, and since positive X is defined as forwards, that's moving 1 m/s backwards
        //     swerve.applyRequest(() ->
        //         drive.withVelocityX(MetersPerSecond.of(-1.0))
        //     ).schedule();

        //     if (movementTimer.hasElapsed(1.0)) {
        //         movementTimer.stop();

        //         // zero it to make it stop
        //         swerve.applyRequest(() ->
        //             drive.withVelocityX(MetersPerSecond.of(0.0))
        //         ).schedule();
        //     }
        // }

        if (autoCommandIndex == -1) return;

        if (autoCommandTimer.hasElapsed(autoCommands.get(autoCommandIndex).seconds)) {
            autoCommandTimer.reset();
            autoCommands.get(autoCommandIndex).onFinished.run();

            if (autoCommands.size() > autoCommandIndex + 1) {
                autoCommandTimer.start();
                autoCommandIndex++;
            } else {
                autoCommandIndex = -1;
            }
        } else {
            autoCommands.get(autoCommandIndex).duringPeriodic.run();
        }
    }

    // init
    public void onDisabled() {
        // compProcessor.elevatorHeightMeters = 0.0;
    }

    public void autonomousInit() {
        // the resetRotation function takes an angle from the blue alliance perspective, so the value we pass to it varies by the alliance color
        if (DriverStation.getAlliance().isPresent()) {
            DriverStation.Alliance alliance = DriverStation.getAlliance().get();

            if (alliance == DriverStation.Alliance.Red) {
                // * the side with the coral arm should be facing away from and perpendicular to the blue alliance driver stations
                swerve.resetRotation(Rotation2d.kZero);
            } else {
                // * the side with the coral arm should be facing towards and perpendicular to the blue alliance driver stations
                swerve.resetRotation(Rotation2d.k180deg);
            }
        } else {
            swerve.resetRotation(Rotation2d.kZero);
        }

        // update timers
        // movementTimer.reset();
        // movementTimer.start();

        autoCommands = chooser.getSelected();

        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        // if (autos.autonomousCommand() != null)
        //     autos.autonomousCommand().schedule();
        
        if (autoCommands.size() == 0) {
            autoCommandIndex = -1;
            return;
        } else {
            autoCommandIndex = 0;
        }
        
        autoCommandTimer.reset();
        autoCommandTimer.start();
    }

    private record Triple(double seconds, Runnable duringPeriodic, Runnable onFinished) {};
}
