package frc.robot.input.comp;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.constants.AlgaeArmConstants;
import frc.lib.constants.CoralArmConstants;
import frc.lib.constants.ElevatorConstants;
import frc.lib.constants.SwerveConstants;
import frc.robot.commands.ElevatorHeightCommand;
import frc.robot.commands.algaearm.AlgaeArmAngleCommand;
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

public class CompInputProcessor {
    // controllers
    public final CommandXboxController driver;
    public final CommandXboxController operator;

    // subsystems
    public final Swerve swerve;
    public final Elevator elevator;

    public final CoralArm coralArm;
    public final CoralWheel coralWheel;

    public final AlgaeArm algaeArm;
    public final AlgaeWheel algaeWheel;

    // state
    public double elevatorHeightMeters = 0.0;
    
    // initialization
    public CompInputProcessor(
        Swerve swerve,
        Elevator elevator,

        CoralArm coralArm,
        CoralWheel coralWheel,

        AlgaeArm algaeArm,
        AlgaeWheel algaeWheel,

        CommandXboxController driver,
        CommandXboxController operator
    ) {
        this.driver = driver;
        this.operator = operator;

        this.swerve = swerve;
        this.elevator = elevator;

        this.coralArm = coralArm;
        this.coralWheel = coralWheel;

        this.algaeArm = algaeArm;
        this.algaeWheel = algaeWheel;
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

        Trigger noBumpers = lb.and(rb).negate();

        // commands
        driver.a().and(noBumpers).onTrue(new CoralIntakeCommand(coralWheel));
        driver.b().and(noBumpers).onTrue(new CoralEjectCommand(coralWheel));

        driver.a().and(lb).onTrue(new ParallelCommandGroup(
            new ElevatorHeightCommand(elevator, ElevatorConstants.CORAL_STATION_INTAKE_HEIGHT),
            new CoralArmAngleCommand(coralArm, CoralArmConstants.STATION_INTAKE_ANGLE)
        ));

        driver.x().and(rb).onTrue(new ParallelCommandGroup(
            new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L1_CORAL_SCORE_HEIGHT),
            new CoralArmAngleCommand(coralArm, CoralArmConstants.L1_SCORE_ANGLE)
        ));

        driver.a().and(rb).onTrue(new ParallelCommandGroup(
            new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L2_CORAL_SCORE_HEIGHT),
            new CoralArmAngleCommand(coralArm, CoralArmConstants.L23_SCORE_ANGLE)
        ));

        driver.b().and(rb).onTrue(new ParallelCommandGroup(
            new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L3_CORAL_SCORE_HEIGHT),
            new CoralArmAngleCommand(coralArm, CoralArmConstants.L23_SCORE_ANGLE)
        ));

        driver.y().and(rb).onTrue(new ParallelCommandGroup(
            new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L4_CORAL_SCORE_HEIGHT),
            new CoralArmAngleCommand(coralArm, CoralArmConstants.L4_SCORE_ANGLE)
        ));
    }

    public void configureOperatorTriggers() {
        // triggers
        Trigger lb = operator.leftBumper();
        Trigger rb = operator.rightBumper();

        Trigger noBumpers = lb.and(rb).negate();

        // commands
        operator.a().and(noBumpers).onTrue(new AlgaeIntakeCommand(algaeWheel));
        operator.b().and(noBumpers).onTrue(new AlgaeEjectCommand(algaeWheel));

        operator.a().and(lb).onTrue(new ParallelCommandGroup(
            new ElevatorHeightCommand(elevator, ElevatorConstants.GROUND),
            new AlgaeArmAngleCommand(algaeArm, AlgaeArmConstants.GROUND_INTAKE_ANGLE)
        ));

        operator.b().and(lb).onTrue(new ParallelCommandGroup(
            new ElevatorHeightCommand(elevator, ElevatorConstants.PROCESSOR_ALGAE_SCORE_HEIGHT),
            new AlgaeArmAngleCommand(algaeArm, AlgaeArmConstants.PROCESSOR_EJECT_ANGLE)
        ));

        operator.a().and(rb).onTrue(new ParallelCommandGroup(
            new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L2_ALGAE_INTAKE_HEIGHT),
            new AlgaeArmAngleCommand(algaeArm, AlgaeArmConstants.L23_INTAKE_ANGLE)
        ));

        operator.b().and(rb).onTrue(new ParallelCommandGroup(
            new ElevatorHeightCommand(elevator, ElevatorConstants.REEF_L3_ALGAE_INTAKE_HEIGHT),
            new AlgaeArmAngleCommand(algaeArm, AlgaeArmConstants.L23_INTAKE_ANGLE)
        ));

        // default commands
        operator.leftTrigger().and(operator.rightTrigger()).whileTrue(new InstantCommand(() -> {
            // determine input values
            // fully up means -1, which is unintuitive, so it requires inversion
            double controllerVerticalSpeed = -operator.getLeftY();

            // process them
            controllerVerticalSpeed = MathUtil.applyDeadband(controllerVerticalSpeed, ElevatorConstants.SPEED_DEADBAND);
            controllerVerticalSpeed *= 0.01;

            elevatorHeightMeters += controllerVerticalSpeed;

            elevator.updateSetpoint(Meters.of(elevatorHeightMeters));
        }, elevator));
    }

    // - defaults
    public void configureDefaults() {
        swerve.setDefaultCommand(new InstantCommand(() -> {
            // calculate controller chassis speeds
            // - convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC
            // - fully right means 1, which is intuitive
            double xSpeed = driver.getLeftX();

            // - fully up means -1, which is unintuitive, so it requires inversion
            double ySpeed = -driver.getLeftY();

            // - fully right means 1, which is positive; however, in WPILib, positive rotation means CCW rotation, and moving the joystick right is generally
            // - associated with CW rotation, so it requires inversion
            double rotSpeed = -driver.getRightX();

            // calculate actual chassis speeds
            // - deadband x, y, and rotation speeds to avoid accidental idle drift
            xSpeed = MathUtil.applyDeadband(xSpeed, SwerveConstants.TRANSLATIONAL_SPEED_DEADBAND);
            ySpeed = MathUtil.applyDeadband(ySpeed, SwerveConstants.TRANSLATIONAL_SPEED_DEADBAND);
            rotSpeed = MathUtil.applyDeadband(rotSpeed, SwerveConstants.ROTATIONAL_SPEED_DEADBAND);

            // - convert velocity values from the unitless range [-1, 1] to the range with units [-max speed, max speed]
            LinearVelocity xVel = SwerveConstants.TRANSLATIONAL_MAX_SPEED.times(xSpeed);
            LinearVelocity yVel = SwerveConstants.TRANSLATIONAL_MAX_SPEED.times(ySpeed);
            AngularVelocity angVel = SwerveConstants.ROTATIONAL_MAX_SPEED.times(rotSpeed);

            swerve.driveRobotRelative(xVel, yVel, angVel);
        }, swerve));
    }
}
