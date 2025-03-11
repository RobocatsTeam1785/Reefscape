package frc.robot.commands.swerve;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/** a command to align the robot's wheels to a specified robot-relative angle */
public class AlignWheelsCommand extends Command {
    private final Angle alignTo;
    private final Swerve swerve;

    public AlignWheelsCommand(Angle alignTo, Swerve swerve) {
        this.alignTo = alignTo;
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.align(alignTo);
    }

    @Override
    public boolean isFinished() {
        return swerve.totalTurnErrorRadians() < 0.1;
    }
}
