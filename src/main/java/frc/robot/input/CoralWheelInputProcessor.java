package frc.robot.input;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.input.InputProcessor;
import frc.lib.mode.ModeState;
import frc.robot.modes.CoralWheelMode;
import frc.robot.subsystems.CoralWheel;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class CoralWheelInputProcessor extends InputProcessor {
    // subsystems
    private final CoralWheel wheel;

    // controllers
    private final CommandXboxController driver;

    // modes
    private final ModeState<CoralWheelMode> state;

    // debug values
    // TODO implement a less hacky method of testing speeds
    private static final double DEBUG_SPEED_METERS_PER_SECOND = 0.0;

    private static final double SPEED_INCREMENT_METERS_PER_SECOND = 0.1;
    @Logged private double manualSpeedMetersPerSecond = 0.0;

    // TODO make a read-only version of ModeState to disallow registering mode switches in an InputProcessor, outside of SubsystemInputProcessor
    public CoralWheelInputProcessor(final CoralWheel wheel, final CommandXboxController driver, final ModeState<CoralWheelMode> state) {
        this.wheel = wheel;
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
        driver.b().and(state.noSwitchesActive()).and(isActive).and(state.is(CoralWheelMode.MANUAL)).onTrue(new InstantCommand(() -> {
            wheel.updateSetpoint(MetersPerSecond.of(DEBUG_SPEED_METERS_PER_SECOND));
        }, wheel));

        driver.y().and(state.noSwitchesActive()).and(isActive).and(state.is(CoralWheelMode.MANUAL)).onTrue(new InstantCommand(() -> {
            manualSpeedMetersPerSecond += SPEED_INCREMENT_METERS_PER_SECOND;
        }));

        driver.a().and(state.noSwitchesActive()).and(isActive).and(state.is(CoralWheelMode.MANUAL)).onTrue(new InstantCommand(() -> {
            manualSpeedMetersPerSecond -= SPEED_INCREMENT_METERS_PER_SECOND;
        }));

        driver.x().and(state.noSwitchesActive()).and(isActive).and(state.is(CoralWheelMode.MANUAL)).onTrue(new InstantCommand(() -> {
            wheel.updateSetpoint(MetersPerSecond.of(manualSpeedMetersPerSecond));
        }, wheel));

        // state-based
    }

    @Override
    public void configureDefaults(Map<Subsystem, Map<ModeState<?>, Command>> defaults) {}
}
