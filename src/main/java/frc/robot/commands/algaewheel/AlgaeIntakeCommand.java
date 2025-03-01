package frc.robot.commands.algaewheel;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeWheel;

public class AlgaeIntakeCommand extends Command {
    public final AlgaeWheel wheel;
    public boolean started = false;
    public long startTimeMs;

    public AlgaeIntakeCommand(AlgaeWheel wheel) {
        this.wheel = wheel;
        addRequirements(wheel);
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        
    }

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        wheel.updateSetpoint(MetersPerSecond.of(20.0));
        started = true;
        startTimeMs = System.currentTimeMillis();
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
        wheel.updateSetpoint(MetersPerSecond.of(0.0));
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return started && (System.currentTimeMillis() - startTimeMs) > 100 && wheel.leftEncoder.getVelocity() < 0.1 && wheel.rightEncoder.getVelocity() < 0.1;
    }
}
