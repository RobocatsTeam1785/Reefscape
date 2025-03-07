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

        // driver.a().and(lb.negate()).and(rb.negate()).onTrue(new InstantCommand(() -> coralWheel.updateSetpoint(MetersPerSecond.of(5.0)), coralWheel));
        // driver.b().and(lb.negate()).and(rb.negate()).onTrue(new InstantCommand(() -> coralWheel.updateSetpoint(MetersPerSecond.of(-3.0)), coralWheel));
        // driver.y().and(lb.negate()).and(rb.negate()).onTrue(new InstantCommand(() -> coralWheel.updateSetpoint(MetersPerSecond.of(0.0)), coralWheel));

        // left bumper
        // driver.a().and(lb).onTrue(new Command() {
        //     @Override
        //     public void initialize() {
        //         coralArm.updateVoltage(Volts.of(1.0));
        //     }

        //     @Override
        //     public void end(boolean interrupted) {
        //         coralArm.updateVoltage(Volts.of(0.0));
        //     }

        //     @Override
        //     public boolean isFinished() {
        //         return coralArm.hexPosition().minus(Degrees.of(30.0)).abs(Radians) < 0.1;
        //     }
        // });

        // driver.b().and(lb).onTrue(new Command() {
        //     @Override
        //     public void initialize() {
        //         coralArm.updateVoltage(Volts.of(-1.0));
        //     }

        //     @Override
        //     public void end(boolean interrupted) {
        //         coralArm.updateVoltage(Volts.of(0.0));
        //     }

        //     @Override
        //     public boolean isFinished() {
        //         return coralArm.hexPosition().minus(CoralArmConstants.MIN_ANGLE).abs(Radians) < 0.1;
        //     }
        // });
        
        // driver.a().and(lb).onTrue(new ParallelCommandGroup(
        //     // new ElevatorHeightCommand(elevator, ElevatorConstants.CORAL_STATION_INTAKE_HEIGHT),
        //     new CoralArmAngleCommand(coralArm, CoralArmConstants.STATION_INTAKE_ANGLE)
        // ));

        // righit bumper
        // driver.x().and(rb).onTrue(new ParallelCommandGroup(
        //     // new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L1_CORAL_SCORE_HEIGHT),
        //     new CoralArmAngleCommand(coralArm, CoralArmConstants.L1_SCORE_ANGLE)
        // ));

        // driver.a().and(rb).onTrue(new ParallelCommandGroup(
        //     // new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L2_CORAL_SCORE_HEIGHT),
        //     new CoralArmAngleCommand(coralArm, CoralArmConstants.L23_SCORE_ANGLE)
        // ));

        // driver.b().and(rb).onTrue(new ParallelCommandGroup(
        //     // new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L3_CORAL_SCORE_HEIGHT),
        //     new CoralArmAngleCommand(coralArm, CoralArmConstants.L23_SCORE_ANGLE)
        // ));

        // driver.y().and(rb).onTrue(new ParallelCommandGroup(
        //     // new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L4_CORAL_SCORE_HEIGHT),
        //     new CoralArmAngleCommand(coralArm, CoralArmConstants.L4_SCORE_ANGLE)
        // ));
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
                if (operator.leftTrigger().getAsBoolean()) {
                    algaeEnabled = !algaeEnabled;
                } else {
                    coralEnabled = !coralEnabled;

                    // if (coralEnabled) {
                    //     coralArmSetpoint = CoralArmConstants.STATION_INTAKE_ANGLE;
                    // } else {
                    //     coralArmSetpoint = null;
                    // }
                }

                i = 10;
            }

            @Override
            public void execute() {
                if (!operator.leftTrigger().getAsBoolean()) {
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
                if (operator.leftTrigger().getAsBoolean()) {
                    algaeEnabled = !algaeEnabled;
                } else {
                    coralEnabled = !coralEnabled;
                }

                i = 10;
            }

            @Override
            public void execute() {
                if (!operator.leftTrigger().getAsBoolean()) {
                    if (coralEnabled) {
                        coralWheel.updateSetpoint(MetersPerSecond.of(-3.0));
                    } else {
                        coralWheel.updateVoltage(Volts.of(0.0));
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

        operator.leftTrigger().onChange(new InstantCommand(() -> {
            if (operator.leftTrigger().getAsBoolean()) {
                // newly pressed
                coralArm.updateVoltage(Volts.of(0.0));
                coralWheel.updateVoltage(Volts.of(0.0));
                coralEnabled = false;
            } else {
                // // newly unpressed
                // algaeArm.updateVoltage(Volts.of(0.0));
                // // algaeWheel.updateVoltage(Volts.of(0.0));
                // algaeEnabled = false;
            }
        }, coralArm/*, algaeArm */));

        // var x = new Command() {
        //     private boolean enabled = false;

        //     @Override
        //     public void initialize() {
        //         enabled = !enabled;
        //     }

        //     @Override
        //     public void execute() {
        //         if (enabled) {
        //             coralWheel.updateSetpoint(MetersPerSecond.of(5.0));
        //         } else {
        //             coralWheel.updateSetpoint(MetersPerSecond.of(0.0));
        //         }
        //     }

        //     @Override
        //     public boolean isFinished() {
        //         return true;
        //     }
        // };
        // operator.x().onTrue(x);

        // var y = new Command() {
        //     private boolean enabled = false;

        //     @Override
        //     public void initialize() {
        //         enabled = !enabled;
        //     }

        //     @Override
        //     public void execute() {
        //         if (enabled) {
        //             coralWheel.updateSetpoint(MetersPerSecond.of(-3.0));
        //         } else {
        //             coralWheel.updateSetpoint(MetersPerSecond.of(0.0));
        //         }
        //     }

        //     @Override
        //     public boolean isFinished() {
        //         return true;
        //     }
        // };
        // operator.y().onTrue(y);

        // commands
        // operator.a().and(noBumpers).onTrue(new InstantCommand(() -> algaeWheel.updateSetpoint(MetersPerSecond.of(5.0)), algaeWheel));
        // operator.b().and(noBumpers).onTrue(new InstantCommand(() -> algaeWheel.updateSetpoint(MetersPerSecond.of(-5.0)), algaeWheel));
        // operator.y().and(noBumpers).onTrue(new InstantCommand(() -> algaeWheel.updateSetpoint(MetersPerSecond.of(0.0)), algaeWheel));

        // left bumper
        // operator.a().and(lb).onTrue(new ParallelCommandGroup(
        //     // new ElevatorHeightCommand(elevator, ElevatorConstants.GROUND),
        //     new AlgaeArmAngleCommand(algaeArm, AlgaeArmConstants.GROUND_INTAKE_ANGLE)
        // ));

        // operator.b().and(lb).onTrue(new ParallelCommandGroup(
        //     // new ElevatorHeightCommand(elevator, ElevatorConstants.PROCESSOR_ALGAE_SCORE_HEIGHT),
        //     new AlgaeArmAngleCommand(algaeArm, AlgaeArmConstants.PROCESSOR_EJECT_ANGLE)
        // ));

        // operator.x().and(lb).onTrue(new ParallelCommandGroup(
        //     // new ElevatorHeightCommand(elevator, ElevatorConstants.PROCESSOR_ALGAE_SCORE_HEIGHT),
        //     new AlgaeArmAngleCommand(algaeArm, AlgaeArmConstants.MAX_ANGLE)
        // ));

        // operator.leftTrigger().onTrue(new InstantCommand(() -> algaeArm.updateVoltage(Volts.of(2.0)), algaeArm));
        // operator.rightTrigger().onTrue(new InstantCommand(() -> algaeArm.updateVoltage(Volts.of(-2.0)), algaeArm));
        // operator.x().onTrue(new InstantCommand(() -> algaeArm.updateVoltage(Volts.of(0.0)), algaeArm));

        // right bumper
        // operator.a().and(rb).onTrue(new ParallelCommandGroup(
        //     // new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L2_ALGAE_INTAKE_HEIGHT),
        //     new AlgaeArmAngleCommand(algaeArm, AlgaeArmConstants.L23_INTAKE_ANGLE)
        // ));

        // operator.b().and(rb).onTrue(new ParallelCommandGroup(
        //     // new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L3_ALGAE_INTAKE_HEIGHT),
        //     new AlgaeArmAngleCommand(algaeArm, AlgaeArmConstants.L23_INTAKE_ANGLE)
        // ));

        // default commands
        // operator.leftTrigger().and(operator.rightTrigger()).whileTrue(new RepeatCommand(new InstantCommand(() -> {
        //     // determine input values
        //     // fully up means -1, which is unintuitive, so it requires inversion
        //     double controllerVerticalSpeed = -operator.getLeftY();

        //     // process them
        //     controllerVerticalSpeed = MathUtil.applyDeadband(controllerVerticalSpeed, ElevatorConstants.SPEED_DEADBAND);
        //     controllerVerticalSpeed *= 0.1;
        //     System.out.println("Controller vertical speed: " + controllerVerticalSpeed);

        //     elevatorHeightMeters += controllerVerticalSpeed;

        //     elevator.updateSetpoint(Meters.of(elevatorHeightMeters));
        // }, elevator)));
    }

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
                voltage *= 2.5;
            }

            elevator.updateVoltage(Volts.of(voltage));
        }, elevator));

        coralArm.setDefaultCommand(new InstantCommand(() -> {
            if (Robot.inAutoMode) return;

            if (coralArmSetpoint != null) {
                if (coralArm.hexPosition().minus(coralArmSetpoint).abs(Degrees) < 5.0) {
                    coralArm.updateVoltage(Volts.of(0.1));
                } else {
                    if (coralArm.hexPosition().gt(coralArmSetpoint)) {
                        coralArm.updateVoltage(Volts.of(-1.0));
                    } else if (coralArm.hexPosition().lt(coralArmSetpoint)) {
                        coralArm.updateVoltage(Volts.of(1.0));
                    }
                }
            } else {
                if (!operator.leftTrigger().getAsBoolean()) {
                    double voltage = -operator.getRightY();
                    voltage *= 1.0;
                    voltage = MathUtil.applyDeadband(voltage, 0.1);
    
                    coralArm.updateVoltage(Volts.of(voltage));
                }
            }
        }, coralArm));

        // algaeArm.setDefaultCommand(new InstantCommand(() -> {
        //     if (Robot.inAutoMode) return;

        //     if (operator.leftTrigger().getAsBoolean()) {
        //         double voltage = -operator.getRightY();
        //         voltage *= 2.0;
        //         voltage = MathUtil.applyDeadband(voltage, 0.1);

        //         algaeArm.updateVoltage(Volts.of(voltage));
        //     } else {
        //         // // when the operator isn't directly controlling the algae arm, make it keep itself up by applying small amounts of voltage if it falls down
        //         // if (algaeArm.hexPosition().lt(AlgaeArmConstants.MAX_ANGLE)) {
        //         //     algaeArm.updateVoltage(Volts.of(0.5));
        //         // } else {
        //         //     algaeArm.updateVoltage(Volts.of(0.0));
        //         // }
        //     }
        // }, algaeArm));
    }

    public void periodic() {
        elevator.periodic();
        coralArm.periodic();
        // algaeArm.periodic();
    }
}
