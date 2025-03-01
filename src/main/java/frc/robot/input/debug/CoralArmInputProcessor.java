package frc.robot.input.debug;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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
import frc.robot.modes.CoralArmMode;
import frc.robot.subsystems.CoralArm;

public class CoralArmInputProcessor extends InputProcessor implements Sendable {
    // subsystems
    private final CoralArm arm;

    // controllers
    private final CommandXboxController driver;

    // modes
    private final ModeState<CoralArmMode> state;

    // control
    private final JoystickModuleParams defaultParams;

    private final JoystickModule positionModule;
    private final JoystickModule velocityModule;
    private final JoystickModule voltageModule;

    /** if JOYSTICK_DEADBAND is x, then controller joystick values in the range [-x, x] get reduced to zero */
    private static final double JOYSTICK_DEADBAND = 0.15;

    private static final double BUTTON_POSITION_RESET_RADIANS = 0.0;
    private static final double BUTTON_VELOCITY_RESET_VOLTS = 0.0;
    private static final double BUTTON_VELOCITY_RESET_RADIANS_PER_SECOND = 0.0;

    // TODO make a read-only version of ModeState to disallow registering mode switches in an InputProcessor, outside of SubsystemInputProcessor
    public CoralArmInputProcessor(final CoralArm arm, final CommandXboxController driver, final ModeState<CoralArmMode> state, Function<ModeState<?>, BooleanSupplier> isModeActive) {
        super(isModeActive);

        this.arm = arm;
        this.driver = driver;
        this.state = state;

        this.defaultParams = new JoystickModuleParams(arm, isModeActive.apply(state), state.noSwitchesActive(), state.is(CoralArmMode.DEBUG), JOYSTICK_DEADBAND);

        this.positionModule = new JoystickModule(
            new ControlModule(value -> arm.updateSetpoint(Radians.of(value)), BUTTON_POSITION_RESET_RADIANS),
            defaultParams.let(params -> {
                params.defaultDeadband = 0.0;
            })
        );
        
        this.velocityModule = new JoystickModule(defaultParams, new ControlModule(value -> arm.updateSetpoint(RadiansPerSecond.of(value)), BUTTON_VELOCITY_RESET_RADIANS_PER_SECOND));
        this.voltageModule = new JoystickModule(defaultParams, new ControlModule(
            value -> arm.updateVoltage(Volts.of(value)),
            BUTTON_VELOCITY_RESET_VOLTS,

            0.0,

            0.0,
            5.0
        ));
    }

    @Override
    public void configureTriggers() {
        positionModule.configureSetValueButton(driver.x());
        velocityModule.configureSetValueButton(driver.y());
        voltageModule.configureSetValueButton(driver.a());

        // resetting voltage to zero functions as a reset for all three modules
        voltageModule.configureResetButton(driver.b());
    }

    @Override
    public void configureDefaults(Map<Subsystem, Map<ModeState<?>, Command>> defaults) {
        if (!defaults.containsKey(arm)) defaults.put(arm, new HashMap<>());

        Map<ModeState<?>, Command> commands = defaults.get(arm);

        commands.put(state, state.selectRunnable(Map.of(
            CoralArmMode.DEBUG, this::driveViaModules
        ), arm));
    }

    // driving
    // driving
    public void driveViaModules() {
        boolean leftBumperDown = driver.leftBumper().getAsBoolean();
        boolean rightBumperDown = driver.rightBumper().getAsBoolean();

        // positive, by default, means downwards, so we're inverting it to make upwards positive
        double value = -driver.getRightY();

        if (leftBumperDown && rightBumperDown) {
            // value is interpreted as radians, after range shifting
            positionModule.driveJoystick(value);
        } else if (leftBumperDown && !rightBumperDown) {
            // value is interpreted as radians/s, after range shifting
            velocityModule.driveJoystick(value);
        } else if (!leftBumperDown && rightBumperDown) {
            // value is interpreted as volts, after range shifting
            voltageModule.driveJoystick(value);
        }
    }

    // network tables
    @Override
    public void initSendable(SendableBuilder builder) {
        positionModule.configureSendable(builder, "Position ");
        velocityModule.configureSendable(builder, "Velocity ");
        voltageModule.configureSendable(builder, "Voltage ");
    }

    // periodic
    @Override
    public void periodic() {}
}
