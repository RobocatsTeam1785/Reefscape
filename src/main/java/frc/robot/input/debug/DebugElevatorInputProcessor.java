package frc.robot.input.debug;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.constants.ElevatorConstants;
import frc.lib.input.InputProcessor;
import frc.lib.input.module.ControlModule;
import frc.lib.input.module.JoystickModule;
import frc.lib.input.module.JoystickModuleParams;
import frc.lib.mode.ModeState;
import frc.robot.modes.ElevatorMode;
import frc.robot.subsystems.Elevator;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class DebugElevatorInputProcessor extends InputProcessor implements Sendable {
    // subsystems
    public final Elevator elevator;

    // controllers
    public final CommandXboxController driver;

    // modes
    public final ModeState<ElevatorMode> state;

    // control
    public final JoystickModuleParams defaultParams;

    public final JoystickModule positionModule;
    public final JoystickModule velocityModule;
    public final JoystickModule voltageModule;

    /** if JOYSTICK_DEADBAND is x, then controller joystick values in the range [-x, x] get reduced to zero */
    public static final double JOYSTICK_DEADBAND = 0.15;

    public static final double BUTTON_POSITION_RESET_METERS = 0.0;
    public static final double BUTTON_VELOCITY_RESET_VOLTS = 0.0;
    public static final double BUTTON_VELOCITY_RESET_METERS_PER_SECOND = 0.0;

    @Logged public double currentHeight = 0.0;

    // TODO make a read-only version of ModeState to disallow registering mode switches in an InputProcessor, outside of SubsystemInputProcessor
    public DebugElevatorInputProcessor(final Elevator elevator, final CommandXboxController driver, final ModeState<ElevatorMode> state, Function<ModeState<?>, BooleanSupplier> isModeActive) {
        super(isModeActive);

        this.elevator = elevator;
        this.driver = driver;
        this.state = state;

        // modules
        this.defaultParams = new JoystickModuleParams(elevator, isModeActive.apply(state), state.noSwitchesActive(), state.is(ElevatorMode.DEBUG), JOYSTICK_DEADBAND);

        // - both
        this.positionModule = new JoystickModule(
            new ControlModule(value -> elevator.updateSetpoint(Meters.of(value)), BUTTON_POSITION_RESET_METERS),
            defaultParams.let(params -> {
                params.defaultDeadband = 0.0;
            })
        );

        this.velocityModule = new JoystickModule(defaultParams, new ControlModule(value -> elevator.updateSetpoint(MetersPerSecond.of(value)), BUTTON_VELOCITY_RESET_METERS_PER_SECOND));
        this.voltageModule = new JoystickModule(defaultParams, new ControlModule(
            value -> elevator.updateVoltage(Volts.of(value)),
            BUTTON_VELOCITY_RESET_VOLTS,

            0.0,

            0.0,
            // ! be VERY careful when using right bumper + right joystick in elevator mode - you WILL break the robot if you aren't
            8.0
        ));
    }

    @Override
    public void configureTriggers() {
        positionModule.configureSetValueButton(driver.x());
        velocityModule.configureSetValueButton(driver.y());
        voltageModule.configureSetValueButton(driver.a());

        // resetting voltage to zero functions as a reset for all three modules
        voltageModule.configureResetButton(driver.b());

        driver.leftTrigger(0.5).whileTrue(new InstantCommand(() -> {
            elevator.updateSetpoint(Feet.of(0.0));
        }, elevator));

        driver.rightTrigger(0.5).whileTrue(new InstantCommand(() -> {
            elevator.updateSetpoint(Feet.of(1.0));
        }, elevator));
    }

    @Override
    public void configureDefaults(Map<Subsystem, Map<ModeState<?>, Command>> defaults) {
        if (!defaults.containsKey(elevator)) defaults.put(elevator, new HashMap<>());

        Map<ModeState<?>, Command> commands = defaults.get(elevator);

        commands.put(state, state.selectRunnable(Map.of(
            ElevatorMode.MANUAL, this::driveManual,
            ElevatorMode.DEBUG, this::driveViaModules,
            ElevatorMode.VARIABLE_HEIGHT, this::driveVariableHeight
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
        // ! max speed is 1000 m/s so that's dangerous
        // elevator.updateSetpoint(verticalSpeed);
    }

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

    public void driveVariableHeight() {
        // determine input values
        // fully up means -1, which is unintuitive, so it requires inversion
        double controllerVerticalSpeed = -driver.getLeftY();

        // process them
        controllerVerticalSpeed = MathUtil.applyDeadband(controllerVerticalSpeed, ElevatorConstants.SPEED_DEADBAND);
        controllerVerticalSpeed *= 0.01;

        currentHeight += controllerVerticalSpeed;

        elevator.updateSetpoint(Meters.of(currentHeight));
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
