package frc.robot.input.comp;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.constants.ControlConstants;
import frc.lib.constants.CoralArmConstants;
import frc.lib.constants.SwerveConstants;
import frc.lib.input.MasterInputProcessor;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralWheel;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Swerve;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class CompInputProcessor extends MasterInputProcessor {
    // controllers
    public final CommandXboxController driver;
    public final CommandXboxController operator;

    // subsystems
    public final CommandSwerveDrivetrain swerve;
    public final Elevator elevator;

    public final CoralArm coralArm;
    public final CoralWheel coralWheel;

    public final Climber climber;

    // state
    @Logged public double elevatorHeightMeters = 0.0;

    public boolean coralIfTrueAlgaeIfFalse = true;

    public Angle coralArmSetpoint = null;

    public boolean speedHalvedToggle = false;
    public boolean speedHalvedButton = false;

    public Angle coralArmIntakeAngle = CoralArmConstants.STATION_INTAKE_ANGLE;

    // tuner swerve
    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentricFacingAngle driveRobotRelativeFacingAngle = new SwerveRequest.RobotCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withHeadingPID(12.5, 0.0, 0.5)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric driveFieldRelative = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.times(SwerveConstants.SLOW_TRANSLATIONAL_SPEED_DEADBAND))
            .withRotationalDeadband(SwerveConstants.ROBOT_ROTATIONAL_MAX_SPEED.times(SwerveConstants.SLOW_ROTATIONAL_SPEED_DEADBAND)) // Add a X% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobotRelative = new SwerveRequest.RobotCentric()
            .withDeadband(SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.times(SwerveConstants.TRANSLATIONAL_SPEED_DEADBAND))
            .withRotationalDeadband(SwerveConstants.ROBOT_ROTATIONAL_MAX_SPEED.times(SwerveConstants.ROTATIONAL_SPEED_DEADBAND)) // Add a X% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.in(MetersPerSecond));
    
    // initialization
    public CompInputProcessor(
        CommandSwerveDrivetrain swerve,
        Elevator elevator,

        CoralArm coralArm,
        CoralWheel coralWheel,

        Climber climber,

        int driverPort,
        int operatorPort
    ) {
        this.driver = new CommandXboxController(driverPort);
        this.operator = new CommandXboxController(operatorPort);

        this.swerve = swerve;
        this.elevator = elevator;

        this.coralArm = coralArm;
        this.coralWheel = coralWheel;

        this.climber = climber;
    }

    // configuration
    @Override
    public void configure() {
        configureDriverTriggers();
        configureOperatorTriggers();

        configureDefaults();

        swerve.registerTelemetry(logger::telemeterize);
    }

    // - triggers
    public void configureDriverTriggers() {
        // triggers
        Trigger lb = driver.leftBumper();
        Trigger rb = driver.rightBumper();

        // commands
        // reset the field-centric heading on a button press
        driver.a().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        driver.b().onTrue(new InstantCommand(() -> {
            speedHalvedToggle = !speedHalvedToggle;
        }));

        driver.rightBumper().onTrue(new InstantCommand(() -> {
            speedHalvedButton = true;
        }));

        driver.rightBumper().onFalse(new InstantCommand(() -> {
            speedHalvedButton = false;
        }));
    }

    // TODO move this to top
    private boolean coralEnabled = false, algaeEnabled = false;

    public void configureOperatorTriggers() {
        // triggers
        var a = new Command() {
            private int i;

            @Override
            public void initialize() {
                coralEnabled = !coralEnabled;

                // if (coralEnabled && !operator.leftTrigger().getAsBoolean()) {
                //     coralArmSetpoint = coralArmIntakeAngle;
                // } else {
                //     coralArmSetpoint = null;
                // }

                i = 10;
            }

            @Override
            public void execute() {
                if (coralEnabled) {
                    coralWheel.updateSetpoint(MetersPerSecond.of(5.0));
                } else {
                    coralWheel.updateVoltage(Volts.of(0.0));
                }

                i--;
            }

            @Override
            public boolean isFinished() {
                return i == 0;
            }
        };
        operator.a().onTrue(a);

        var b = new Command() {
            private int i;

            @Override
            public void initialize() {
                coralEnabled = !coralEnabled;

                // if (coralEnabled && !operator.leftTrigger().getAsBoolean()) {
                //     coralArmSetpoint = CoralArmConstants.L4_SCORE_ANGLE;
                // } else {
                //     coralArmSetpoint = null;
                // }

                i = 10;
            }

            @Override
            public void execute() {
                // if (!operator.leftTrigger().getAsBoolean()) {
                //     if (coralArmSetpoint != null && coralEnabled && coralArm.hexPosition().minus(coralArmSetpoint).abs(Degrees) < CoralArmConstants.MARGIN_OF_ERROR.in(Degrees)) {
                //         coralWheel.updateSetpoint(MetersPerSecond.of(-3.0));
                //     } else {
                //         coralWheel.updateVoltage(Volts.of(0.0));

                //         // if we're outside of the accepted range, we'll reach it eventually, since the compensation is currently programmed correctly,
                //         // so we can counteract the counter decreasing until we reach the range
                //         i++;
                //     }
                // } else {
                    if (coralEnabled) {
                        coralWheel.updateSetpoint(MetersPerSecond.of(-3.0));
                    } else {
                        coralWheel.updateVoltage(Volts.of(0.0));
                    }
                // }

                i--;
            }

            @Override
            public boolean isFinished() {
                return i == 0;
            }
        };
        operator.b().onTrue(b);

        // operator.x().onTrue(new InstantCommand(() -> {
        //     if (coralArmIntakeAngle.minus(CoralArmConstants.STATION_INTAKE_ANGLE).abs(Degrees) < 1e-6) {
        //         coralArmIntakeAngle = CoralArmConstants.STATION_INTAKE_ANGLE_ALT;

        //         if (coralArmSetpoint != null && coralArmSetpoint.minus(CoralArmConstants.STATION_INTAKE_ANGLE).abs(Degrees) < 1e-6) {
        //             coralArmSetpoint = coralArmIntakeAngle;
        //         }
        //     } else {
        //         coralArmIntakeAngle = CoralArmConstants.STATION_INTAKE_ANGLE;

        //         if (coralArmSetpoint != null && coralArmSetpoint.minus(CoralArmConstants.STATION_INTAKE_ANGLE_ALT).abs(Degrees) < 1e-6) {
        //             coralArmSetpoint = coralArmIntakeAngle;
        //         }
        //     }
        // }));
    }

    double coralArmIdleVoltage = 0.2;

    // - defaults
    public void configureDefaults() {
        swerve.setDefaultCommand(
            // Drivetrain will execute this command periodically
            swerve.applyRequest(() -> {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                // Drive forward with negative Y (forward)
                double xSpeed = -driver.getLeftY();

                // Drive left with negative X (left)
                double ySpeed = -driver.getLeftX();

                // Drive counterclockwise with negative X (left)
                double rotSpeed = -driver.getRightX();

                // SmartDashboard.putNumber("cip x speed", xSpeed);
                // SmartDashboard.putNumber("cip y speed", ySpeed);
                // SmartDashboard.putNumber("cip rot speed", rotSpeed);

                // calculate actual chassis speeds

                // SmartDashboard.putNumber("cip db x speed", xSpeed);
                // SmartDashboard.putNumber("cip db y speed", ySpeed);
                // SmartDashboard.putNumber("cip db rot speed", rotSpeed);

                // ! deadbanding and max speed are already accounted for in the declaration of the `drive` command in this class, so look at the property declarations for details

                if (speedHalvedButton || speedHalvedToggle) {
                    xSpeed *= 0.5;
                    ySpeed *= 0.5;
                }

                // SmartDashboard.putNumber("cip x vel m|s", xVel.in(MetersPerSecond));
                // SmartDashboard.putNumber("cip y vel m|s", yVel.in(MetersPerSecond));
                // SmartDashboard.putNumber("cip rot vel rad|s", angVel.in(RadiansPerSecond));

                SmartDashboard.putBoolean("LT auto-align active", driver.leftTrigger().getAsBoolean() && RobotContainer.lastTagPose != null);

                // ! note: if both triggers are pressed, this implementation will always choose the left trigger behavior
                if (driver.leftTrigger().getAsBoolean()) {
                    RobotCentricFacingAngle request = driveRobotRelativeFacingAngle
                        .withVelocityX(SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.times(xSpeed * SwerveConstants.ROBOT_SLOW_MODE_FRACTION)) // Drive forward with negative Y (forward)
                        .withVelocityY(SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.times(ySpeed * SwerveConstants.ROBOT_SLOW_MODE_FRACTION)); // Drive left with negative X (left)
                    
                    if (RobotContainer.lastTagPose != null) {
                        request = request.withTargetDirection(RobotContainer.lastTagPose.getRotation().toRotation2d().rotateBy(Rotation2d.k180deg));
                    }

                    return request;
                } else if (driver.rightTrigger().getAsBoolean()) {
                    return driveRobotRelative
                        .withVelocityX(SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.times(xSpeed * SwerveConstants.ROBOT_SLOW_MODE_FRACTION)) // Drive forward with negative Y (forward)
                        .withVelocityY(SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.times(ySpeed * SwerveConstants.ROBOT_SLOW_MODE_FRACTION)) // Drive left with negative X (left)
                        .withRotationalRate(SwerveConstants.ROBOT_ROTATIONAL_MAX_SPEED.times(rotSpeed)); // Drive counterclockwise with negative X (left)
                } else {
                    return driveFieldRelative
                        .withVelocityX(SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.times(xSpeed)) // Drive forward with negative Y (forward)
                        .withVelocityY(SwerveConstants.ROBOT_TRANSLATIONAL_MAX_SPEED.times(ySpeed)) // Drive left with negative X (left)
                        .withRotationalRate(SwerveConstants.ROBOT_ROTATIONAL_MAX_SPEED.times(rotSpeed)); // Drive counterclockwise with negative X (left)
                }
            })
        );

        // if the operator right trigger is pressed, we control the voltage of the climber
        // if the operator right trigger is not pressed, we control the voltage of the elevator
        elevator.setDefaultCommand(new InstantCommand(() -> {
            if (Robot.inAutoMode) return;

            if (operator.rightTrigger().getAsBoolean()) {
                elevator.updateVoltage(Volts.zero());
                return;
            }

            double voltage = -operator.getLeftY();
            voltage = MathUtil.applyDeadband(voltage, 0.1);

            if (voltage > 0.0) {
                voltage *= 5.0;
            } else if (voltage < 0.0) {
                if (elevator.leftHeight().in(Meters) < 0.3) {
                    voltage *= 1.25;
                } else {
                    voltage *= 5.0;
                }
            }

            elevator.updateVoltage(Volts.of(voltage));
        }, elevator));

        // ! be VERY careful when using this - the climber lifts the entire robot up, so mishandling this has a very real risk of damaging the entire robot to a severe degree
        climber.setDefaultCommand(new InstantCommand(() -> {
            if (!operator.rightTrigger().getAsBoolean()) {
                climber.updateVoltage(0.0);
                return;
            }

            double volts = -operator.getLeftY(); // back is positive, we want back is negative
            volts = MathUtil.applyDeadband(volts, ControlConstants.CLIMBER_VOLTAGE_CONTROL_DEADBAND);

            if (volts > 0.0) {
                volts *= ControlConstants.CLIMBER_VOLTAGE_CONTROL_POSITIVE_FACTOR;
            } else if (volts < 0.0) {
                volts *= ControlConstants.CLIMBER_VOLTAGE_CONTROL_NEGATIVE_FACTOR;
            }

            climber.updateVoltage(volts);
        }, climber));

        coralArm.setDefaultCommand(new InstantCommand(() -> {
            if (Robot.inAutoMode) return;

            // if (coralArmSetpoint != null) {
            //     if (coralArm.hexPosition().minus(coralArmSetpoint).abs(Degrees) < CoralArmConstants.MARGIN_OF_ERROR.in(Degrees)) {
            //         coralArm.updateVoltage(Volts.of(coralArmIdleVoltage));
            //     } else {
            //         if (coralArm.hexPosition().gt(coralArmSetpoint)) {
            //             coralArm.updateVoltage(Volts.of(-0.5));
            //         } else if (coralArm.hexPosition().lt(coralArmSetpoint)) {
            //             coralArm.updateVoltage(Volts.of(0.5));
            //         }
            //     }
            // } else {
            //     if (true) {
                    double voltage = -operator.getRightY();
                    voltage *= 1.0;
                    voltage = MathUtil.applyDeadband(voltage, 0.1);
    
                    coralArm.updateVoltage(Volts.of(voltage));
            //     }
            // }
        }, coralArm));
    }

    @Override
    public void periodic() {
        elevator.periodic();
        coralArm.periodic();
    }
}
