package frc.robot.commands.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class AlignWheelsForRotationCommand extends Command {
    /** a rotational speed value small enough to effectively perform no robot rotation while rotating the wheels into the correct rotation */
    private static final AngularVelocity EPSILON = RadiansPerSecond.of(0.001);

    private final Swerve swerve;

    public AlignWheelsForRotationCommand(Swerve swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.driveRobotRelative(MetersPerSecond.zero(), MetersPerSecond.zero(), EPSILON);
    }

    @Override
    public boolean isFinished() {
        return swerve.totalTurnErrorRadians() < 0.1;
    }
}
