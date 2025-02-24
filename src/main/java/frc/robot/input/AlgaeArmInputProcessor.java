package frc.robot.input;

import static edu.wpi.first.units.Units.Degrees;

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
import frc.robot.modes.AlgaeArmMode;
import frc.robot.subsystems.AlgaeArm;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class AlgaeArmInputProcessor extends InputProcessor {
    // subsystems
    private final AlgaeArm arm;

    // controllers
    private final CommandXboxController driver;

    // modes
    private final ModeState<AlgaeArmMode> state;

    // debug values
    // TODO implement a less hacky method of testing angles
    private static final double DEBUG_ANGLE_DEGREES = 0.0;

    private static final double ANGLE_INCREMENT_DEGREES = 10.0;
    @Logged private double manualAngle = DEBUG_ANGLE_DEGREES;

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
        driver.b().and(state.noSwitchesActive()).and(isActive).and(state.is(AlgaeArmMode.MANUAL)).onTrue(new InstantCommand(() -> {
            arm.updateSetpoint(Degrees.of(DEBUG_ANGLE_DEGREES));
        }, arm));

        // ! be very careful that specified angles do not exceed the operating range, as exceeding it risks damaging the robot
        driver.y().and(state.noSwitchesActive()).and(isActive).and(state.is(AlgaeArmMode.MANUAL)).onTrue(new InstantCommand(() -> {
            manualAngle += ANGLE_INCREMENT_DEGREES;
        }));

        driver.a().and(state.noSwitchesActive()).and(isActive).and(state.is(AlgaeArmMode.MANUAL)).onTrue(new InstantCommand(() -> {
            manualAngle -= ANGLE_INCREMENT_DEGREES;
        }));

        driver.x().and(state.noSwitchesActive()).and(isActive).and(state.is(AlgaeArmMode.MANUAL)).onTrue(new InstantCommand(() -> {
            arm.updateSetpoint(Degrees.of(manualAngle));
        }, arm));

        // state-based
    }

    @Override
    public void configureDefaults(Map<Subsystem, Map<ModeState<?>, Command>> defaults) {}
}
