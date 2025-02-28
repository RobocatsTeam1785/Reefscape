package frc.lib.input.module;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickModuleParams {
    public Subsystem subsystem;

    public BooleanSupplier isActive;
    public Trigger noSwitchesActive;
    public Trigger isRelevantMode;

    public double defaultDeadband;

    public JoystickModuleParams(
        Subsystem subsystem,

        BooleanSupplier isActive,
        Trigger noSwitchesActive,
        Trigger isRelevantMode,

        double defaultDeadband
    ) {
        this.subsystem = subsystem;

        this.isActive = isActive;
        this.noSwitchesActive = noSwitchesActive;
        this.isRelevantMode = isRelevantMode;

        this.defaultDeadband = defaultDeadband;
    }

    public JoystickModuleParams copy() {
        return new JoystickModuleParams(subsystem, isActive, noSwitchesActive, isRelevantMode, defaultDeadband);
    }

    /** applies the specified modifications to a copy of this object and returns the copy */
    public JoystickModuleParams let(Consumer<JoystickModuleParams> mutator) {
        JoystickModuleParams copy = copy();
        mutator.accept(copy);

        return copy;
    }
}