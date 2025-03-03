package frc.robot.commands.coralarm;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class CoralArmAngleCommand extends Command {
    public final CoralArm arm;
    public final Angle angle;

    public CoralArmAngleCommand(CoralArm arm, Angle angle) {
        this.arm = arm;
        this.angle = angle;

        addRequirements(arm);
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        
    }

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        arm.updateSetpoint(angle);
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or
     * when it interrupted/canceled.
     *
     * <p>Do not schedule commands here that share requirements with this command. Use {@link
     * #andThen(Command...)} instead.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return arm.hexPosition().minus(angle).abs(Radians) < 0.2;
    }
}
