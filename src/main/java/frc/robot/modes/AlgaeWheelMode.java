package frc.robot.modes;

import edu.wpi.first.epilogue.Logged;
import frc.lib.mode.Mode;

@Logged
public enum AlgaeWheelMode implements Mode {
    /**
     * pressing the b button sets the velocity of both wheels to a constant value that can be hot-reloaded for tuning
     * <p>
     * pressing the y button increments the speed by 0.1 m/s, and pressing the a button decrements it by 0.1 m/s
     */
    MANUAL,

    /**
     * pressing the b button sets the velocity of the left wheel to a constant value that can be hot-reloaded for tuning
     * <p>
     * pressing the y button increments the speed by 0.1 m/s, and pressing the a button decrements it by 0.1 m/s
     */
    MANUAL_LEFT_ONLY,

    /**
     * pressing the b button sets the velocity of the right wheel to a constant value that can be hot-reloaded for tuning
     * <p>
     * pressing the y button increments the speed by 0.1 m/s, and pressing the a button decrements it by 0.1 m/s
     */
    MANUAL_RIGHT_ONLY;
}
