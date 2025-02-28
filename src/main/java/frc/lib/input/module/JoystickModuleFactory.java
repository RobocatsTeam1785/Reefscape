package frc.lib.input.module;

import java.util.function.Consumer;

public class JoystickModuleFactory {
    private final JoystickModuleParams defaultParams;

    // initialization
    public JoystickModuleFactory(JoystickModuleParams defaultParams) {
        this.defaultParams = defaultParams;
    }

    // state modification
    public JoystickModule module(ControlModule module, Consumer<JoystickModuleParams> mutator) {
        return new JoystickModule(module, defaultParams.let(mutator));
    }
}
