package frc.lib.utility;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.mode.Mode;
import frc.lib.mode.ModeState;

public class CommandUtils {
    /** sets the default command of owner to r, with owner as the only dependency */
    public static void setDefault(Subsystem owner, Runnable r) {
        owner.setDefaultCommand(new InstantCommand(r, owner));
    }

    /** sets the default command of a subsystem to a selection of per-mode runnables with the subsystem as a requirement */
    public static <S extends Subsystem, M extends Mode> void selectDefault(S subsystem, ModeState<M> state, Map<M, Runnable> runnables) {
        subsystem.setDefaultCommand(state.selectRunnable(runnables, subsystem));
    }
}
