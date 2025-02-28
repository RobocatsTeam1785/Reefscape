package frc.robot.modes;

import edu.wpi.first.epilogue.Logged;
import frc.lib.mode.Mode;

@Logged
public enum ElevatorMode implements Mode {
    /** default left joystick controlled up and down movement */
    MANUAL,

    // TODO add description based on input processor
    DEBUG;
}
