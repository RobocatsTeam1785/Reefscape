package frc.robot.input;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.constants.ElevatorConstants;
import frc.lib.input.InputProcessor;
import frc.lib.mode.ModeState;
import frc.robot.modes.ElevatorMode;
import frc.robot.subsystems.Elevator;

public class ElevatorInputProcessor extends InputProcessor {
    // subsystems
    private final Elevator elevator;

    // controllers
    private final CommandXboxController driver;

    // modes
    private final ModeState<ElevatorMode> state;

    // TODO make a read-only version of ModeState to disallow registering mode switches in an InputProcessor, outside of SubsystemInputProcessor
    public ElevatorInputProcessor(final Elevator elevator, final CommandXboxController driver, final ModeState<ElevatorMode> state) {
        this.elevator = elevator;
        this.driver = driver;
        this.state = state;
    }

    @Override
    public void configureTriggers(Function<ModeState<?>, BooleanSupplier> isModeActive) {
        // buttons
        // state-based
    }

    @Override
    public void configureDefaults(Map<Subsystem, Map<ModeState<?>, Command>> defaults) {
        if (!defaults.containsKey(elevator)) defaults.put(elevator, new HashMap<>());

        Map<ModeState<?>, Command> commands = defaults.get(elevator);

        commands.put(state, state.selectRunnable(Map.of(
            ElevatorMode.MANUAL, this::driveManual
        ), elevator));
    }

    // driving
    /** moves the elevator up and down using the left joystick, where upwards joystick movement means upwards elevator movement and vice versa */
    public void driveManual() {
        // determine input values
        // fully up means -1, which is unintuitive, so it requires inversion
        double controllerVerticalSpeed = -driver.getLeftY();

        // process them
        controllerVerticalSpeed = MathUtil.applyDeadband(controllerVerticalSpeed, ElevatorConstants.SPEED_DEADBAND);

        // apply units
        LinearVelocity verticalSpeed = ElevatorConstants.MAX_SPEED.times(controllerVerticalSpeed);

        // drive
        elevator.updateSetpoint(verticalSpeed);
    }
}
