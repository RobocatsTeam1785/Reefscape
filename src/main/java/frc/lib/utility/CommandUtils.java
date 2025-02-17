package frc.lib.utility;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandUtils {
    /** sets the default command of owner to r, with owner as the only dependency */
    public static void setDefault(Subsystem owner, Runnable r) {
        owner.setDefaultCommand(new InstantCommand(r, owner));
    }
}
