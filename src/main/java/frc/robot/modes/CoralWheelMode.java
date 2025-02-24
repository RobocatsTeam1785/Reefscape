package frc.robot.modes;

import edu.wpi.first.epilogue.Logged;
import frc.lib.mode.Mode;

@Logged
public enum CoralWheelMode implements Mode {
    /**
     * pressing the b button sets the velocity to a constant value that can be hot-reloaded for tuning
     * <p>
     * pressing the y button increments the speed by 0.1 m/s, pressing the a button decrements it by 0.1 m/s, and pressing x applies the change
     */
    MANUAL;
}
