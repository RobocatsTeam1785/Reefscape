package frc.robot.modes;

public enum DriveMode {
    /** normal swerve driving with simultaneous translation and rotation */
    SWERVE(-1),
    
    /** turn-only driving that aligns to a specific angle */
    ALIGN(-1),

    // module-only control
    FL_ONLY(0), FR_ONLY(1), BL_ONLY(2), BR_ONLY(3);

    /** the swerve module this mode refers to, if this is a single-module-only mode, and -1 if not */
    public final int id;

    private DriveMode(int id) {
        this.id = id;
    }

    /** whether this drive mode controls only a single module */
    public boolean oneModuleOnly() {
        return id >= 0;
    }
}
