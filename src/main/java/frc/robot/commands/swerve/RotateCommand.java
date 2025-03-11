package frc.robot.commands.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

// TODO genericize this into the Swerve subsystem class as creating a new ProfiledPIDController every time the command is used is unnecessary
public class RotateCommand extends Command {
    // goals
    private final double rotateToRadians;

    // subsystems
    private final Swerve swerve;

    // loop control
    private static final boolean TUNE = true;

    private static final double
        ROBOT_ROTATION_ONLY_KP = 0.0,
        ROBOT_ROTATION_ONLY_KI = 0.0,
        ROBOT_ROTATION_ONLY_KD = 0.0;

    private final ProfiledPIDController controller;

    public RotateCommand(Angle rotateTo, Swerve swerve) {
        // instance properties
        this.rotateToRadians = rotateTo.in(Radians);
        this.swerve = swerve;

        this.controller = new ProfiledPIDController(ROBOT_ROTATION_ONLY_KP, ROBOT_ROTATION_ONLY_KI, ROBOT_ROTATION_ONLY_KD, SwerveConstants.ROBOT_TURN_PROFILE_CONSTRAINTS_RADIANS);

        // requirements
        addRequirements(swerve);

        // dependencies
        beforeStarting(new AlignWheelsForRotationCommand(swerve));
    }

    // ! this command assumes the robot is not rotating when this command is initialized
    @Override
    public void initialize() {
        double yawDegrees = swerve.navX2.getAngle();
        double yawRadians = Units.degreesToRadians(yawDegrees);

        controller.reset(yawRadians);
    }

    @Override
    public void execute() {
        if (TUNE) {
            controller.setPID(ROBOT_ROTATION_ONLY_KP, ROBOT_ROTATION_ONLY_KI, ROBOT_ROTATION_ONLY_KD);
        }

        double yawDegrees = swerve.navX2.getAngle();
        double yawRadians = Units.degreesToRadians(yawDegrees);

        double speedRadiansPerSecond = controller.calculate(yawRadians, rotateToRadians);

        swerve.driveRobotRelative(MetersPerSecond.zero(), MetersPerSecond.zero(), RadiansPerSecond.of(speedRadiansPerSecond));
    }
}
