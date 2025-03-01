package frc.lib.input;

import static edu.wpi.first.units.Units.Radians;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.mode.ModeState;

/** an object that transforms controller inputs into subsystem setpoints */
public abstract class InputProcessor {
    public final Function<ModeState<?>, BooleanSupplier> isModeActive;

    public InputProcessor(Function<ModeState<?>, BooleanSupplier> isModeActive) {
        this.isModeActive = isModeActive;
    }

    /** code that executes every time the main robot loop executes */
    public abstract void periodic();

    /** configure trigger-based commands */
    public abstract void configureTriggers();

    /** add per-mode subsystem default commands */
    public abstract void configureDefaults(Map<Subsystem, Map<ModeState<?>, Command>> defaults);

    // utility functions
    /**
     * the rotation designated by the driver left joystick, where fully right means zero, and counterclockwise is positive
     * in turn, "zero" is defined as the yaw the robot had when the navX was last zeroed
     */
    public static Angle leftAlignAngle(CommandXboxController driver) {
        // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC

        // fully right means 1, which is intuitive in ENU
        double x = driver.getLeftX();

        // fully up means -1, which is unintuitive in ENU, so it requires inversion
        double y = -driver.getLeftY();

        double angleRadians = Math.atan2(y, x);
        return Radians.of(angleRadians);
    }

    /**
     * the rotation designated by the driver right joystick, where fully right means zero, and counterclockwise is positive
     * in turn, "zero" is defined as the yaw the robot had when the navX was last zeroed
     */
    public static Angle rightAlignAngle(CommandXboxController driver) {
        // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC

        // fully right means 1, which is intuitive in ENU
        double x = driver.getRightX();

        // fully up means -1, which is unintuitive in ENU, so it requires inversion
        double y = -driver.getRightY();

        double angleRadians = Math.atan2(y, x);
        return Radians.of(angleRadians);
    }
}
