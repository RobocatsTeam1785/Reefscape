package frc.robot.modes;

import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import frc.lib.mode.Mode;
import frc.robot.subsystems.Swerve;

@Logged
public enum DriveMode implements Mode {
    /** normal swerve driving with simultaneous translation and rotation */
    SWERVE,
    
    /** turn-only driving that aligns to a specific angle */
    ALIGN,

    // module-only control
    FL_ONLY(Swerve.ModuleId.FL), FR_ONLY(Swerve.ModuleId.FR), BL_ONLY(Swerve.ModuleId.BL), BR_ONLY(Swerve.ModuleId.BR);

    /** the swerve module this mode refers to, if this is a single-module-only mode, and -1 if not */
    @NotLogged public final Optional<Swerve.ModuleId> id;

    private DriveMode(Swerve.ModuleId id) {
        this.id = Optional.of(id);
    }

    private DriveMode() {
        this.id = Optional.empty();
    }

    /** whether this drive mode controls only a single module */
    public boolean oneModuleOnly() {
        return id.isPresent();
    }
}
