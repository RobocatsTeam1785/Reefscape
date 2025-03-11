package frc.robot.input.comp;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.constants.AlgaeArmConstants;
import frc.lib.constants.CoralArmConstants;
import frc.lib.constants.ElevatorConstants;
import frc.lib.constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.commands.ElevatorHeightCommand;
import frc.robot.commands.algaearm.AlgaeArmAngleCommand;
import frc.robot.commands.algaearm.AlgaeArmDownCommand;
import frc.robot.commands.algaewheel.AlgaeEjectCommand;
import frc.robot.commands.algaewheel.AlgaeIntakeCommand;
import frc.robot.commands.coralarm.CoralArmAngleCommand;
import frc.robot.commands.coralwheel.CoralEjectCommand;
import frc.robot.commands.coralwheel.CoralIntakeCommand;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeWheel;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralWheel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class CompInputProcessor {
    // controllers
    public final CommandXboxController driver;
    public final CommandXboxController operator;

    // subsystems
    public final Swerve swerve;
    public final Elevator elevator;

    public final CoralArm coralArm;
    public final CoralWheel coralWheel;

    // public final AlgaeArm algaeArm;
    // public final AlgaeWheel algaeWheel;

    // state
    @Logged public double elevatorHeightMeters = 0.0;

    public boolean coralIfTrueAlgaeIfFalse = true;

    public Angle coralArmSetpoint = null;

    public boolean speedHalvedToggle = false;
    public boolean speedHalvedButton = false;

    public Angle coralArmIntakeAngle = CoralArmConstants.STATION_INTAKE_ANGLE;
    
    // initialization
    public CompInputProcessor(
        Swerve swerve,
        Elevator elevator,

        CoralArm coralArm,
        CoralWheel coralWheel,

        // AlgaeArm algaeArm,
        // AlgaeWheel algaeWheel,

        CommandXboxController driver,
        CommandXboxController operator
    ) {
        this.driver = driver;
        this.operator = operator;

        this.swerve = swerve;
        this.elevator = elevator;

        this.coralArm = coralArm;
        this.coralWheel = coralWheel;

        // this.algaeArm = algaeArm;
        // this.algaeWheel = algaeWheel;
    }

    // configuration
    public void configure() {
        configureDriverTriggers();
        configureOperatorTriggers();

        configureDefaults();
    }

    // - triggers
    public void configureDriverTriggers() {
        // triggers
        Trigger lb = driver.leftBumper();
        Trigger rb = driver.rightBumper();

        // commands
        driver.a().onTrue(new InstantCommand(() -> {
            swerve.navX2.zeroYaw();
        }, swerve) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        });

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
        Trigger lb = operator.leftBumper();
        Trigger rb = operator.rightBumper();

        Trigger noBumpers = lb.and(rb).negate();

        var a = new Command() {
            private int i;

            @Override
            public void initialize() {
                if (false) {
                    algaeEnabled = !algaeEnabled;
                } else {
                    coralEnabled = !coralEnabled;

                    if (coralEnabled && !operator.leftTrigger().getAsBoolean()) {
                        coralArmSetpoint = coralArmIntakeAngle;
                    } else {
                        coralArmSetpoint = null;
                    }
                }

                i = 10;
            }

            @Override
            public void execute() {
                if (true) {
                    if (coralEnabled) {
                        coralWheel.updateSetpoint(MetersPerSecond.of(5.0));
                    } else {
                        coralWheel.updateVoltage(Volts.of(0.0));
                    }
                } else {
                    // if (algaeEnabled) {
                    //     algaeWheel.updateSetpoint(MetersPerSecond.of(5.0));
                    // } else {
                    //     algaeWheel.updateVoltage(Volts.of(0.0));
                    // }
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
                if (false) {
                    algaeEnabled = !algaeEnabled;
                } else {
                    coralEnabled = !coralEnabled;

                    if (coralEnabled && !operator.leftTrigger().getAsBoolean()) {
                        coralArmSetpoint = CoralArmConstants.L4_SCORE_ANGLE;
                    } else {
                        coralArmSetpoint = null;
                    }
                }

                i = 10;
            }

            @Override
            public void execute() {
                if (true) {
                    if (!operator.leftTrigger().getAsBoolean()) {
                        if (coralArmSetpoint != null && coralEnabled && coralArm.hexPosition().minus(coralArmSetpoint).abs(Degrees) < CoralArmConstants.MARGIN_OF_ERROR.in(Degrees)) {
                            coralWheel.updateSetpoint(MetersPerSecond.of(-3.0));
                        } else {
                            coralWheel.updateVoltage(Volts.of(0.0));

                            // if we're outside of the accepted range, we'll reach it eventually, since the compensation is currently programmed correctly,
                            // so we can counteract the counter decreasing until we reach the range
                            i++;
                        }
                    } else {
                        if (coralEnabled) {
                            coralWheel.updateSetpoint(MetersPerSecond.of(-3.0));
                        } else {
                            coralWheel.updateVoltage(Volts.of(0.0));
                        }
                    }
                } else {
                    // if (algaeEnabled) {
                    //     algaeWheel.updateSetpoint(MetersPerSecond.of(-5.0));
                    // } else {
                    //     algaeWheel.updateVoltage(Volts.of(0.0));
                    // }
                }

                i--;
            }

            @Override
            public boolean isFinished() {
                return i == 0;
            }
        };
        operator.b().onTrue(b);

        operator.x().onTrue(new InstantCommand(() -> {
            if (coralArmIntakeAngle.minus(CoralArmConstants.STATION_INTAKE_ANGLE).abs(Degrees) < 1e-6) {
                coralArmIntakeAngle = CoralArmConstants.STATION_INTAKE_ANGLE_ALT;

                if (coralArmSetpoint != null && coralArmSetpoint.minus(CoralArmConstants.STATION_INTAKE_ANGLE).abs(Degrees) < 1e-6) {
                    coralArmSetpoint = coralArmIntakeAngle;
                }
            } else {
                coralArmIntakeAngle = CoralArmConstants.STATION_INTAKE_ANGLE;

                if (coralArmSetpoint != null && coralArmSetpoint.minus(CoralArmConstants.STATION_INTAKE_ANGLE_ALT).abs(Degrees) < 1e-6) {
                    coralArmSetpoint = coralArmIntakeAngle;
                }
            }
        }));
    }

    double coralArmIdleVoltage = 0.2;

    // - defaults
    public void configureDefaults() {
        swerve.setDefaultCommand(new InstantCommand(() -> {
            if (Robot.inAutoMode) return;

            // calculate controller chassis speeds
            // - convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC
            // - fully right means 1, which is intuitive
            double oxSpeed = driver.getLeftX();

            // - fully up means -1, which is unintuitive, so it requires inversion
            double oySpeed = -driver.getLeftY();

            double xSpeed = oySpeed;
            double ySpeed = -oxSpeed;

            // - fully right means 1, which is positive; however, in WPILib, positive rotation means CCW rotation, and moving the joystick right is generally
            // - associated with CW rotation, so it requires inversion
            double rotSpeed = -driver.getRightX();

            // SmartDashboard.putNumber("cip x speed", xSpeed);
            // SmartDashboard.putNumber("cip y speed", ySpeed);
            // SmartDashboard.putNumber("cip rot speed", rotSpeed);

            // calculate actual chassis speeds
            // - deadband x, y, and rotation speeds to avoid accidental idle drift
            xSpeed = MathUtil.applyDeadband(xSpeed, SwerveConstants.TRANSLATIONAL_SPEED_DEADBAND);
            ySpeed = MathUtil.applyDeadband(ySpeed, SwerveConstants.TRANSLATIONAL_SPEED_DEADBAND);
            rotSpeed = MathUtil.applyDeadband(rotSpeed, SwerveConstants.ROTATIONAL_SPEED_DEADBAND);

            // SmartDashboard.putNumber("cip db x speed", xSpeed);
            // SmartDashboard.putNumber("cip db y speed", ySpeed);
            // SmartDashboard.putNumber("cip db rot speed", rotSpeed);

            if (speedHalvedButton || speedHalvedToggle) {
                xSpeed *= 0.5;
                ySpeed *= 0.5;
            }

            // - convert velocity values from the unitless range [-1, 1] to the range with units [-max speed, max speed]
            LinearVelocity xVel = SwerveConstants.TRANSLATIONAL_MAX_SPEED.times(xSpeed);
            LinearVelocity yVel = SwerveConstants.TRANSLATIONAL_MAX_SPEED.times(ySpeed);
            AngularVelocity angVel = SwerveConstants.ROTATIONAL_MAX_SPEED.times(rotSpeed);

            // SmartDashboard.putNumber("cip x vel m|s", xVel.in(MetersPerSecond));
            // SmartDashboard.putNumber("cip y vel m|s", yVel.in(MetersPerSecond));
            // SmartDashboard.putNumber("cip rot vel rad|s", angVel.in(RadiansPerSecond));

            if (swerve.navX2.isConnected() && !swerve.navX2.isCalibrating()) {
                swerve.driveFieldRelative(xVel, yVel, angVel);
            } else {
                swerve.driveRobotRelative(xVel, yVel, angVel);  
            }
        }, swerve));

        elevator.setDefaultCommand(new InstantCommand(() -> {
            if (Robot.inAutoMode) return;

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

        coralArm.setDefaultCommand(new InstantCommand(() -> {
            if (Robot.inAutoMode) return;

            if (coralArmSetpoint != null) {
                if (coralArm.hexPosition().minus(coralArmSetpoint).abs(Degrees) < CoralArmConstants.MARGIN_OF_ERROR.in(Degrees)) {
                    coralArm.updateVoltage(Volts.of(coralArmIdleVoltage));
                } else {
                    if (coralArm.hexPosition().gt(coralArmSetpoint)) {
                        coralArm.updateVoltage(Volts.of(-0.5));
                    } else if (coralArm.hexPosition().lt(coralArmSetpoint)) {
                        coralArm.updateVoltage(Volts.of(0.5));
                    }
                }
            } else {
                if (true) {
                    double voltage = -operator.getRightY();
                    voltage *= 1.0;
                    voltage = MathUtil.applyDeadband(voltage, 0.1);
    
                    coralArm.updateVoltage(Volts.of(voltage));
                }
            }
        }, coralArm));
    }

    public void periodic() {
        elevator.periodic();
        coralArm.periodic();
    }
}
