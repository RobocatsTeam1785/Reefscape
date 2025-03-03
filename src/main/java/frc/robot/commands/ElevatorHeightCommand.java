package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorHeightCommand extends Command {
    public final Elevator elevator;
    public final Distance height;

    public ElevatorHeightCommand(Elevator elevator, Distance height) {
        this.elevator = elevator;
        this.height = height;

        addRequirements(elevator);
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        
    }

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        elevator.updateSetpoint(height);
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
        elevator.updateVoltage(Volts.of(0.0));
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return elevator.leftHeight().minus(height).abs(Meters) < 0.1 && elevator.rightHeight().minus(height).abs(Meters) < 0.1;
    }
}
