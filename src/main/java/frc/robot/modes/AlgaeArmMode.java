package frc.robot.modes;

import edu.wpi.first.epilogue.Logged;
import frc.lib.mode.Mode;

@Logged
public enum AlgaeArmMode implements Mode {
    // TODO update this to the actual configuration in AlgaeArmInputProcessor
    /**
     * pressing the b button sets the angle to a constant value that can be hot-reloaded for tuning
     * <p>
     * pressing the y button increments the angle by 10°, pressing the a button decrements it by 10°, and pressing x applies the change
     */
    DEBUG;
}
