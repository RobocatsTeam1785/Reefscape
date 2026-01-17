package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.constants.AutoConstants;
import frc.lib.constants.RobotConstants;
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

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
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
            .withDriveRequestType(DriveRequestType.Velocity);
    
    private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    
    private final SwerveRequest.ApplyRobotSpeeds autoDrive = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity);

    private final SendableChooser<List<AutoFunction>> chooser;

    @Logged public Pose2d latestVisionPose = new Pose2d();
    public static Pose2d lastTargetPose;

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
            (speeds, feedforwards) -> {
                if (lastTargetPose != null)
                    System.out.println("Distance to targetPose: " + lastTargetPose.minus(swerve.getState().Pose));

                // Apply the control
                swerve.setControl(autoDrive.withSpeeds(speeds));
            },
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
            new CompInputProcessor(swerve, elevator, coralArm, coralWheel, /* algaeArm, algaeWheel, */ climber, 0, 1, this),
            // new ShuffleboardInputProcessor("Control", swerve, elevator, coralArm, coralWheel, algaeArm, algaeWheel),
            // new TestInputProcessor(3, swerve)
        };

        for (MasterInputProcessor processor : processors) {
            processor.configure();
        }

        // autos
        autos = new Autos();
        autos.chooser.addOption("Move 1 Meter Relative", getMoveOneMeterCommand());
        autos.chooser.addOption("Circle Spin", getCircleSpinCommand());

        // timers
        // movementTimer = new Timer();
        // alignTimer = new Timer();

        // auto shuffleboard stuff
        chooser = new SendableChooser<>();
        chooser.setDefaultOption("straight", straightL1Auto);
        chooser.addOption("long horizontal L1", longHorizontalL1Auto);
        chooser.addOption("long horizontal L2", longHorizontalL2Auto);
        chooser.addOption("move", onlyMoveAuto);

        SmartDashboard.putData("Auto Chooser (Manual)", chooser);

    }

    // periodic
    public void robotPeriodic() {
        // --- START SWERVE STATE LOGGING ---
        var state = swerve.getState();

        // 1. Log Pose (The most critical part for PathPlanner)
        SmartDashboard.putNumber("SwerveState/PoseX", state.Pose.getX());
        SmartDashboard.putNumber("SwerveState/PoseY", state.Pose.getY());
        SmartDashboard.putNumber("SwerveState/PoseRot", state.Pose.getRotation().getDegrees());
        
        // Debug Flag: Is the pose corrupt?
        boolean isNaN = Double.isNaN(state.Pose.getX()) || Double.isNaN(state.Pose.getY());
        SmartDashboard.putBoolean("SwerveState/IsPoseNaN", isNaN);

        // 2. Log Speeds
        SmartDashboard.putNumber("SwerveState/SpeedVx", state.Speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("SwerveState/SpeedVy", state.Speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("SwerveState/SpeedOmega", state.Speeds.omegaRadiansPerSecond);

        // 3. Log Raw Inputs/Diagnostics
        SmartDashboard.putNumber("SwerveState/RawHeading", state.RawHeading.getDegrees());
        SmartDashboard.putNumber("SwerveState/Timestamp", state.Timestamp);
        SmartDashboard.putNumber("SwerveState/OdometryFreq", 1.0 / state.OdometryPeriod);
        SmartDashboard.putNumber("SwerveState/FailedDaqs", state.FailedDaqs);

        // 4. Log Module States (assuming 4 modules)
        if (state.ModuleStates != null && state.ModuleTargets != null) {
            for (int i = 0; i < state.ModuleStates.length; i++) {
                String key = "SwerveState/Mod" + i;
                SmartDashboard.putNumber(key + "/CurrentSpeed", state.ModuleStates[i].speedMetersPerSecond);
                SmartDashboard.putNumber(key + "/CurrentAngle", MathUtil.inputModulus(state.ModuleStates[i].angle.getDegrees(), -180, 180));
                SmartDashboard.putNumber(key + "/TargetSpeed", state.ModuleTargets[i].speedMetersPerSecond);
                SmartDashboard.putNumber(key + "/TargetAngle", MathUtil.inputModulus(state.ModuleTargets[i].angle.getDegrees(), -180, 180));
            }
        }
        // --- END SWERVE STATE LOGGING ---

        // add vision measurement and log relative X, Y, and angle and absolute angle
        Pose2d previousPose = swerve.getState().Pose;
        Optional<EstimatedRobotPose> maybeVisionPose = vision.getEstimatedGlobalPose(previousPose);

        if (maybeVisionPose.isPresent()) {
            EstimatedRobotPose visionPose = maybeVisionPose.get();

            Pose2d visionRobotPoseMeters = visionPose.estimatedPose.toPose2d();
            double timestampSeconds = visionPose.timestampSeconds;

            this.latestVisionPose = visionRobotPoseMeters;

            // Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9); 
            swerve.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds); //, visionStdDevs);

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
    private List<AutoFunction> scoreL1Auto = List.of(
        new AutoFunction(
            1.0,
            () -> flickCoralArmUp(1.0),
            () -> {}
        ),
        new AutoFunction(
            1.0,
            () -> fallToAngle(Degrees.of(-33), 0.6),
            () -> {}
        ),
        new AutoFunction(
            1.0,
            () -> spinCoralWheels(-3.0),
            () -> spinCoralWheels(0.0)
        )
    );

    private List<AutoFunction> scoreL2Auto = List.of(
        new AutoFunction(
            1.0,
            () -> flickCoralArmUp(1.0),
            () -> {}
        ),
        new AutoFunction(
            1.0,
            () -> fallToAngle(Degrees.of(0), 0.4),
            () -> {}
        ),
        new AutoFunction(
            1.0,
            () -> spinCoralWheels(-3.0),
            () -> spinCoralWheels(0.0)
        )
    );

    private List<AutoFunction> straightL1Auto = List.of(
        new AutoFunction(
            2.5,
            () -> drive(-1.0, 0.0, 0.0),
            () -> drive(0.0, 0.0, 0.0)
        )
    );

    private List<AutoFunction> longHorizontalL1Auto = List.of(
        new AutoFunction(
            2.5 * 1.33,
            () -> drive(-1.0, 0.0, 0.0),
            () -> drive(0.0, 0.0, 0.0)
        )
    );

    private List<AutoFunction> longHorizontalL2Auto = List.of(
        new AutoFunction(
            1.0,
            () -> flickCoralArmUp(1.0),
            () -> {}
        ),
        new AutoFunction(
            1.0,
            () -> fallToAngle(Degrees.of(-33), 0.6),
            () -> {}
        ),

        new AutoFunction(
            0.2,
            () -> drive(-0.001, 0.0, 0.0),
            () -> drive(0.0, 0.0, 0.0)
        ),

        new AutoFunction(
            2.5 * 1.33 + 0.5,
            () -> drive(-1.0, 0.0, 0.0),
            () -> drive(0.0, 0.0, 0.0)
        ),

        new AutoFunction(
            1.0,
            () -> spinCoralWheels(-3.0),
            () -> spinCoralWheels(0.0)
        ),

        new AutoFunction(
            0.1,
            () -> {},
            () -> {}
        ),

        new AutoFunction(
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

    private final List<AutoFunction> onlyMoveAuto = List.of(
        new AutoFunction(
            1.0,
            () -> drive(-1.0, 0.0, 0.0),
            () -> drive(0.0, 0.0, 0.0)
        )
    );

    private List<AutoFunction> autoCommands = straightL1Auto;

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
        // autos.autonomousCommand().schedule();
        // System.out.println(autos.autonomousCommand());
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
        // // the resetRotation function takes an angle from the blue alliance perspective, so the value we pass to it varies by the alliance color
        // if (DriverStation.getAlliance().isPresent()) {
        //     DriverStation.Alliance alliance = DriverStation.getAlliance().get();

        //     if (alliance == DriverStation.Alliance.Red) {
        //         // * the side with the coral arm should be facing away from and perpendicular to the blue alliance driver stations
        //         swerve.resetRotation(Rotation2d.kZero);
        //     } else {
        //         // * the side with the coral arm should be facing towards and perpendicular to the blue alliance driver stations
        //         swerve.resetRotation(Rotation2d.k180deg);
        //     }
        // } else {
        //     swerve.resetRotation(Rotation2d.kZero);
        // }

        // update timers
        // movementTimer.reset();
        // movementTimer.start();

        autoCommandIndex = -1;

        // autoCommands = chooser.getSelected();

        // DriverStation.reportWarning(autoCommands.toString(), true);
        // System.out.print(autoCommands.toString());

        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        Command pathPlannerCommand = autos.autonomousCommand();

        System.out.println("1" + pathPlannerCommand.getName());

        if (pathPlannerCommand != null) {
            System.out.println("2" + pathPlannerCommand.getName());

            pathPlannerCommand.schedule();
        }
        
        // if (autoCommands.size() == 0) {
        //     autoCommandIndex = -1;
        //     return;
        // } else {
        //     autoCommandIndex = 0;
        // }
        
        // autoCommandTimer.reset();
        // autoCommandTimer.start();
    }

    private record AutoFunction(double seconds, Runnable duringPeriodic, Runnable onFinished) {};

    public Command getMoveOneMeterCommand() {
        return Commands.defer(() -> {
            Pose2d currentPose = swerve.getState().Pose;
            
            // Calculate Target
            Translation2d forwardVec = new Translation2d(1.85, 0.0).rotateBy(currentPose.getRotation());
            Pose2d targetPose = new Pose2d(
                currentPose.getTranslation().plus(forwardVec), 
                currentPose.getRotation()
            );

            // Generate Path
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                new PathConstraints(
                    SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.in(MetersPerSecond),
                    SwerveConstants.ROBOT_TRANSLATIONAL_MAX_ACCELERATION.in(MetersPerSecondPerSecond),
                    SwerveConstants.ROBOT_ROTATIONAL_MAX_SPEED.in(RadiansPerSecond), 
                    SwerveConstants.ROBOT_ROTATIONAL_MAX_ACCELERATION.in(RadiansPerSecondPerSecond)
                ),
                null, 
                new GoalEndState(0.0, targetPose.getRotation())
            );
            path.preventFlipping = true;

            return AutoBuilder.followPath(path);
        }, Set.of(swerve));
    }

    public Command getCircleSpinCommand() {
        return Commands.defer(() -> {
            Pose2d startPose = swerve.getState().Pose;
            Rotation2d startRot = startPose.getRotation();
            
            // Define points for a 5m diameter circle to the left of the robot
            List<Pose2d> pathPoses = new ArrayList<>();
            pathPoses.add(startPose); 
            pathPoses.add(new Pose2d(
                startPose.getTranslation().plus(new Translation2d(2.5, 2.5).rotateBy(startRot)),
                startRot.plus(Rotation2d.fromDegrees(90))
            ));
            pathPoses.add(new Pose2d(
                startPose.getTranslation().plus(new Translation2d(0.0, 5.0).rotateBy(startRot)),
                startRot.plus(Rotation2d.fromDegrees(180))
            ));
            pathPoses.add(new Pose2d(
                startPose.getTranslation().plus(new Translation2d(-2.5, 2.5).rotateBy(startRot)),
                startRot.plus(Rotation2d.fromDegrees(270))
            ));
            pathPoses.add(new Pose2d(
                startPose.getTranslation(),
                startRot
            ));

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathPoses);
            
            PathConstraints constraints = new PathConstraints(
                3.0, 3.0, 
                SwerveConstants.ROBOT_ROTATIONAL_MAX_SPEED.in(RadiansPerSecond),
                SwerveConstants.ROBOT_ROTATIONAL_MAX_ACCELERATION.in(RadiansPerSecondPerSecond)
            );

            // Create the rotation targets list BEFORE constructing the path
            List<RotationTarget> rotationTargets = new ArrayList<>();
            rotationTargets.add(new RotationTarget(1.0, startRot.plus(Rotation2d.fromDegrees(180))));
            rotationTargets.add(new RotationTarget(2.0, startRot.plus(Rotation2d.fromDegrees(360))));
            rotationTargets.add(new RotationTarget(3.0, startRot.plus(Rotation2d.fromDegrees(540))));

            GoalEndState endState = new GoalEndState(0.0, startRot.plus(Rotation2d.fromDegrees(720)));

            // Use the full constructor to pass in the rotation targets
            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                rotationTargets,
                Collections.emptyList(), // pointTowardsZones
                Collections.emptyList(), // constraintZones
                Collections.emptyList(), // eventMarkers
                constraints,
                null, // IdealStartingState
                endState,
                false // reversed
            );
            path.preventFlipping = true;

            return AutoBuilder.followPath(path);
        }, Set.of(swerve));
    }

    /**
     * Generates a deferred command that moves the robot to be flush with the wall of the specified AprilTag.
     * 
     * @param tagId The ID of the AprilTag to align with.
     * @return A command that generates and follows a path to the tag.
     */
    public Command getAlignToTagCommand(int tagId) {
        return Commands.defer(() -> {
            // 1. Retrieve the pose of the AprilTag from the field layout
            Optional<Pose3d> tagPose3d = AutoConstants.layout.getTagPose(tagId);

            if (tagPose3d.isEmpty()) {
                DriverStation.reportError("Attempted to align to invalid tag ID: " + tagId, false);
                return Commands.none();
            }

            Pose2d tagPose = tagPose3d.get().toPose2d();

            // Subtract 5cm to aim "into" the wall
            double offsetCushion = 0.05; 
            double targetDistanceFromTag = (RobotConstants.ROBOT_WIDTH.in(Meters) / 2.0) - offsetCushion;

            Translation2d shiftVector = new Translation2d(targetDistanceFromTag, 0.0).rotateBy(tagPose.getRotation());
            
            // Add the shift to the tag's origin to get the robot's target center
            Translation2d targetTranslation = tagPose.getTranslation().plus(shiftVector);

            // 3. Calculate the target rotation
            // To be flush with the wall, the robot must face the wall.
            // Since the tag faces OUT of the wall, the robot must face the opposite direction (Tag Angle + 180).
            Rotation2d targetRotation = tagPose.getRotation().plus(Rotation2d.fromDegrees(180));

            // 4. Construct Poses
            Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);
            Pose2d currentPose = swerve.getState().Pose;

            lastTargetPose = targetPose;

            // 5. Generate the PathPlanner path on-the-fly
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
            
            PathConstraints constraints = new PathConstraints(
                SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.in(MetersPerSecond),
                SwerveConstants.ROBOT_TRANSLATIONAL_MAX_ACCELERATION.in(MetersPerSecondPerSecond),
                SwerveConstants.ROBOT_ROTATIONAL_MAX_SPEED.in(RadiansPerSecond),
                SwerveConstants.ROBOT_ROTATIONAL_MAX_ACCELERATION.in(RadiansPerSecondPerSecond)
            );

            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // Ideal starting state (null uses robot state)
                new GoalEndState(0.0, targetRotation) // End with 0 velocity, facing the wall
            );
            
            // Prevent the path from trying to drive backwards if the rotation logic is complex
            path.preventFlipping = true;

            // 6. Return the command to follow this path
            return AutoBuilder.followPath(path);
        }, Set.of(swerve));
    }
}
