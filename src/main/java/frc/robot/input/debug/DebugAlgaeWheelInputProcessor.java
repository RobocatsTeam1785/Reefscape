package frc.robot.input.debug;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.input.InputProcessor;
import frc.lib.input.module.ControlModule;
import frc.lib.input.module.JoystickModule;
import frc.lib.input.module.JoystickModuleParams;
import frc.lib.mode.ModeState;
import frc.robot.modes.AlgaeWheelMode;
import frc.robot.subsystems.AlgaeWheel;

public class DebugAlgaeWheelInputProcessor extends InputProcessor implements Sendable {
    // subsystems
    public final AlgaeWheel wheel;

    // controllers
    public final CommandXboxController driver;

    // modes
    public final ModeState<AlgaeWheelMode> state;

    // control
    public final JoystickModuleParams defaultParams;

    public final JoystickModule velocityModule, leftVelocityModule, rightVelocityModule;
    public final JoystickModule voltageModule, leftVoltageModule, rightVoltageModule;

    /** if JOYSTICK_DEADBAND is x, then controller joystick values in the range [-x, x] get reduced to zero */
    public static final double JOYSTICK_DEADBAND = 0.15;

    public static final double BUTTON_VELOCITY_RESET_VOLTS = 0.0;
    public static final double BUTTON_VELOCITY_RESET_METERS_PER_SECOND = 0.0;

    // TODO make a read-only version of ModeState to disallow registering mode switches in an InputProcessor, outside of SubsystemInputProcessor
    public DebugAlgaeWheelInputProcessor(final AlgaeWheel wheel, final CommandXboxController driver, final ModeState<AlgaeWheelMode> state, Function<ModeState<?>, BooleanSupplier> isModeActive) {
        super(isModeActive);

        this.wheel = wheel;
        this.driver = driver;
        this.state = state;

        // modules
        this.defaultParams = new JoystickModuleParams(wheel, isModeActive.apply(state), state.noSwitchesActive(), state.is(AlgaeWheelMode.DEBUG), JOYSTICK_DEADBAND);

        // - both
        this.velocityModule = new JoystickModule(defaultParams, new ControlModule(value -> wheel.updateSetpoint(MetersPerSecond.of(value)), BUTTON_VELOCITY_RESET_METERS_PER_SECOND));
        this.voltageModule = new JoystickModule(defaultParams, new ControlModule(
            value -> wheel.updateVoltage(Volts.of(value)),
            BUTTON_VELOCITY_RESET_VOLTS,

            0.0,

            0.0,
            5.0
        ));

        // - left wheel
        this.leftVelocityModule = new JoystickModule(
            new ControlModule(value -> wheel.updateLeftSetpoint(MetersPerSecond.of(value)), BUTTON_VELOCITY_RESET_METERS_PER_SECOND),
            defaultParams.let(params -> {
                params.isRelevantMode = state.is(AlgaeWheelMode.DEBUG_LEFT_ONLY);
            })
        );

        this.leftVoltageModule = new JoystickModule(
            new ControlModule(value -> wheel.updateLeftVoltage(Volts.of(value)), BUTTON_VELOCITY_RESET_VOLTS),
            defaultParams.let(params -> {
                params.isRelevantMode = state.is(AlgaeWheelMode.DEBUG_LEFT_ONLY);
            })
        );

        // - right wheel
        this.rightVelocityModule = new JoystickModule(
            new ControlModule(value -> wheel.updateRightSetpoint(MetersPerSecond.of(value)), BUTTON_VELOCITY_RESET_METERS_PER_SECOND),
            defaultParams.let(params -> {
                params.isRelevantMode = state.is(AlgaeWheelMode.DEBUG_RIGHT_ONLY);
            })
        );

        this.rightVoltageModule = new JoystickModule(
            new ControlModule(value -> wheel.updateRightVoltage(Volts.of(value)), BUTTON_VELOCITY_RESET_VOLTS),
            defaultParams.let(params -> {
                params.isRelevantMode = state.is(AlgaeWheelMode.DEBUG_RIGHT_ONLY);
            })
        );
    }

    @Override
    public void configureTriggers() {
        velocityModule.configureSetValueButton(driver.y());
        voltageModule.configureSetValueButton(driver.a());

        leftVelocityModule.configureSetValueButton(driver.y());
        leftVoltageModule.configureSetValueButton(driver.a());

        rightVelocityModule.configureSetValueButton(driver.y());
        rightVoltageModule.configureSetValueButton(driver.a());

        // resetting voltage to zero functions as a reset for both modules
        voltageModule.configureResetButton(driver.b());
        leftVoltageModule.configureResetButton(driver.b());
        rightVoltageModule.configureResetButton(driver.b());
    }

    @Override
    public void configureDefaults(Map<Subsystem, Map<ModeState<?>, Command>> defaults) {
        if (!defaults.containsKey(wheel)) defaults.put(wheel, new HashMap<>());

        Map<ModeState<?>, Command> commands = defaults.get(wheel);

        commands.put(state, state.selectRunnable(Map.of(
            AlgaeWheelMode.DEBUG, () -> driveViaModules(velocityModule, voltageModule),
            AlgaeWheelMode.DEBUG_LEFT_ONLY, () -> driveViaModules(leftVelocityModule, leftVoltageModule),
            AlgaeWheelMode.DEBUG_RIGHT_ONLY, () -> driveViaModules(rightVelocityModule, rightVoltageModule)
        ), wheel));
    }

    // driving
    public void driveViaModules(JoystickModule providedVelocityModule, JoystickModule providedVoltageModule) {
        boolean leftBumperDown = driver.leftBumper().getAsBoolean();
        boolean rightBumperDown = driver.rightBumper().getAsBoolean();

        // positive, by default, means downwards, so we're inverting it to make upwards positive
        double value = -driver.getRightY();

        if (leftBumperDown && !rightBumperDown) {
            // value is interpreted as radians/s, after range shifting
            providedVelocityModule.driveJoystick(value);
        } else if (!leftBumperDown && rightBumperDown) {
            // value is interpreted as volts, after range shifting
            providedVoltageModule.driveJoystick(value);
        }
    }

    // network tables
    @Override
    public void initSendable(SendableBuilder builder) {
        velocityModule.configureSendable(builder, "Velocity ");
        voltageModule.configureSendable(builder, "Voltage ");

        leftVelocityModule.configureSendable(builder, "Left Velocity ");
        leftVoltageModule.configureSendable(builder, "Left Voltage ");

        rightVelocityModule.configureSendable(builder, "Right Velocity ");
        rightVoltageModule.configureSendable(builder, "Right Voltage ");
    }

    // periodic
    @Override
    public void periodic() {}
}
