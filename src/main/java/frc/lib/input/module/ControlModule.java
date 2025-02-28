package frc.lib.input.module;

import java.util.function.DoubleConsumer;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class ControlModule {
    /** the consumer that uses the double control value to set mechanism voltage */
    private final DoubleConsumer controlConsumer;

    /** the value to reset control to */
    private final double resetValue;

    /** the user-controlled arbitrary control value */
    @Logged private double value;

    /** the zero point for ranged control */
    @Logged private double base;
    
    /** the distance from the zero point for ranged control */
    @Logged private double magnitude;

    // initialization
    public ControlModule(
        DoubleConsumer controlConsumer,
        double resetValue,

        double defaultValue,

        double defaultBase,
        double defaultMagnitude
    ) {
        this.controlConsumer = controlConsumer;
        this.resetValue = resetValue;

        this.value = defaultValue;
        this.base = defaultBase;
        this.magnitude = defaultMagnitude;
    }

    public ControlModule(DoubleConsumer controlConsumer, double resetValue) {
        this(controlConsumer, resetValue, 0.0, 0.0, 0.0);
    }

    // triggers
    public void configureSetValueButton(Trigger trigger, Subsystem subsystem) {
        trigger.onTrue(new InstantCommand(() -> {
            controlConsumer.accept(value);
        }, subsystem));
    }

    public void configureResetButton(Trigger trigger, Subsystem subsystem) {
        trigger.onTrue(new InstantCommand(() -> {
            controlConsumer.accept(resetValue);
        }, subsystem));
    }

    // defaults
    /**
     * sets an interpolated control value relative to the specified value inside [-1, 1], where 
     * <p>
     * -1 performs <pre>controlConsumer.accept(base - magnitude)</pre>
     * 1 performs <pre>controlConsumer.accept(base + magnitude)</pre>
     * and 0 performs <pre>controlConsumer.accept(base)</pre>
     */
    public void driveWithinRange(double normalizedValue) {
        double interpolated = base + normalizedValue * magnitude;
        controlConsumer.accept(interpolated);
    }

    // network tables
    public void configureSendable(SendableBuilder builder, String prefix) {
        builder.addDoubleProperty(prefix + "Value", () -> value, newValue -> value = newValue);
        builder.addDoubleProperty(prefix + "Base", () -> base, newBase -> base = newBase);
        builder.addDoubleProperty(prefix + "Magnitude", () -> magnitude, newMagnitude -> magnitude = newMagnitude);
    }
}