package frc.lib.input.module;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickModule {
    public final ControlModule module;

    public final Subsystem subsystem;

    public final BooleanSupplier isActive;
    public final Trigger noSwitchesActive;
    public final Trigger isRelevantMode;

    /** if deadband is x, then controller joystick values in the range [-x, x] get reduced to zero */
    public double deadband;

    // initialization
    public JoystickModule(
        JoystickModuleParams params,
        ControlModule module
    ) {
        this.module = module;

        this.subsystem = params.subsystem;

        this.isActive = params.isActive;
        this.noSwitchesActive = params.noSwitchesActive;
        this.isRelevantMode = params.isRelevantMode;

        this.deadband = params.defaultDeadband;
    }

    public JoystickModule(ControlModule module, JoystickModuleParams params) {
        this(params, module);
    }

    // triggers
    public void configureSetValueButton(Trigger button) {
        module.configureSetValueButton(button.and(isActive).and(noSwitchesActive).and(isRelevantMode), subsystem);
    }

    public void configureResetButton(Trigger button) {
        module.configureResetButton(button.and(isActive).and(noSwitchesActive).and(isRelevantMode), subsystem);
    }

    // defaults
    /** sets a control value using the same rules as {@link ControlModule#driveWithinRange(double)} but with the configured deadband applied */
    public void driveJoystick(double joystickValue) {
        joystickValue = MathUtil.applyDeadband(joystickValue, deadband);
        module.driveWithinRange(joystickValue);
    }

    // network tables
    public void configureSendable(SendableBuilder builder, String prefix) {
        builder.addDoubleProperty(prefix + "Deadband", () -> deadband, newDeadband -> deadband = newDeadband);

        module.configureSendable(builder, prefix + "Module ");
    }
}