package frc.robot.input;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.input.InputProcessor;
import frc.lib.mode.ModeState;
import frc.robot.modes.AlgaeArmMode;
import frc.robot.subsystems.AlgaeArm;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class AlgaeArmInputProcessor extends InputProcessor implements Sendable {
    // subsystems
    private final AlgaeArm arm;

    // controllers
    private final CommandXboxController driver;

    // modes
    private final ModeState<AlgaeArmMode> state;

    // debug values
    private static final double BUTTON_POSITION_RESET = 0.0;
    private static final double BUTTON_VELOCITY_RESET = 0.0;

    /** whether to directly control voltage using the velocity value and range, if true, or control the velocity setpoint in radians per second, if false */
    private static boolean directlyControlVoltage = false;

    private static double buttonPositionRadians = 0.0;

    private static double joystickPositionMinimumRadians = 0.0;
    private static double joystickPositionMaximumRadians = 0.0;

    private static double buttonVelocity = 0.0;

    private static double joystickVelocityMinimum = 0.0;
    private static double joystickVelocityMaximum = 0.0;

    // TODO make a read-only version of ModeState to disallow registering mode switches in an InputProcessor, outside of SubsystemInputProcessor
    public AlgaeArmInputProcessor(final AlgaeArm arm, final CommandXboxController driver, final ModeState<AlgaeArmMode> state) {
        this.arm = arm;
        this.driver = driver;
        this.state = state;
    }

    @Override
    public void configureTriggers(Function<ModeState<?>, BooleanSupplier> isModeActive) {
        // defined suppliers
        BooleanSupplier isActive = isModeActive.apply(state);

        // ! all triggers must be logically ANDed with state.noSwitchesActive() and isActive in order to ensure that they
        // ! do not conflict with mode-switching triggers and only run when the current mode state is active

        // buttons
        driver.x().and(state.noSwitchesActive()).and(isActive).and(state.is(AlgaeArmMode.MANUAL)).onTrue(new InstantCommand(() -> {
            arm.updateSetpoint(Radians.of(buttonPositionRadians));
        }, arm));

        driver.y().and(state.noSwitchesActive()).and(isActive).and(state.is(AlgaeArmMode.MANUAL)).onTrue(new InstantCommand(() -> {
            if (directlyControlVoltage) {
                arm.updateVoltage(Volts.of(buttonVelocity));
            } else {
                arm.updateSetpoint(RadiansPerSecond.of(buttonVelocity));
            }
        }, arm));

        driver.a().and(state.noSwitchesActive()).and(isActive).and(state.is(AlgaeArmMode.MANUAL)).onTrue(new InstantCommand(() -> {
            arm.updateSetpoint(Radians.of(BUTTON_POSITION_RESET));
        }, arm));

        driver.b().and(state.noSwitchesActive()).and(isActive).and(state.is(AlgaeArmMode.MANUAL)).onTrue(new InstantCommand(() -> {
            arm.updateSetpoint(RadiansPerSecond.of(BUTTON_VELOCITY_RESET));
        }, arm));

        // state-based
    }

    @Override
    public void configureDefaults(Map<Subsystem, Map<ModeState<?>, Command>> defaults) {
        if (!defaults.containsKey(arm)) defaults.put(arm, new HashMap<>());

        Map<ModeState<?>, Command> commands = defaults.get(arm);

        commands.put(state, state.selectRunnable(Map.of(
            AlgaeArmMode.MANUAL, this::driveWithinJoystickRange
        ), arm));
    }

    // driving
    public void driveWithinJoystickRange() {
        // - fully up means -1, which is unintuitive, so it requires inversion
        double positionRadians = -driver.getLeftY();
        double velocity = -driver.getRightY();

        positionRadians = joystickPositionMinimumRadians + positionRadians * (joystickPositionMaximumRadians - joystickPositionMinimumRadians);
        velocity = joystickVelocityMinimum + velocity * (joystickVelocityMaximum - joystickVelocityMinimum);

        // only apply in-range position setpoints when the left bumper is down
        if (driver.leftBumper().getAsBoolean()) {
            arm.updateSetpoint(Radians.of(positionRadians));
        }

        // only apply in-range velocity setpoints when the right bumper is down
        if (driver.rightBumper().getAsBoolean()) {
            if (directlyControlVoltage) {
                arm.updateVoltage(Volts.of(velocity));
            } else {
                arm.updateSetpoint(RadiansPerSecond.of(velocity));
            }
        }
    }

    // network tables
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("directlyControlVoltage", () -> directlyControlVoltage, value -> directlyControlVoltage = value);

        builder.addDoubleProperty("buttonPositionRadians", () -> buttonPositionRadians, value -> buttonPositionRadians = value);
        builder.addDoubleProperty("buttonVelocity", () -> buttonVelocity, value -> buttonVelocity = value);

        builder.addDoubleProperty("joystickPositionMinimumRadians", () -> joystickPositionMinimumRadians, value -> joystickPositionMinimumRadians = value);
        builder.addDoubleProperty("joystickPositionMaximumRadians", () -> joystickPositionMaximumRadians, value -> joystickPositionMaximumRadians = value);

        builder.addDoubleProperty("joystickVelocityMinimum", () -> joystickVelocityMinimum, value -> joystickVelocityMinimum = value);
        builder.addDoubleProperty("joystickVelocityMaximum", () -> joystickVelocityMaximum, value -> joystickVelocityMaximum = value);
    }

    // periodic
    @Override
    public void periodic() {}
}
