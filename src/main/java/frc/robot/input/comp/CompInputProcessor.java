package frc.robot.input.comp;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.constants.ElevatorConstants;
import frc.lib.constants.SwerveConstants;
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
        configureTriggers();
        configureDefaults();
    }

    // - triggers
    public void configureTriggers() {
        
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

        elevator.setDefaultCommand(new InstantCommand(() -> {
            // determine input values
            // fully up means -1, which is unintuitive, so it requires inversion
            double controllerVerticalSpeed = -operator.getLeftY();

            // process them
            controllerVerticalSpeed = MathUtil.applyDeadband(controllerVerticalSpeed, ElevatorConstants.SPEED_DEADBAND);

            // apply units
            LinearVelocity verticalSpeed = MetersPerSecond.of(1.0).times(controllerVerticalSpeed);

            // drive
            elevator.updateSetpoint(verticalSpeed);
        }, elevator));
    }
}
