package frc.robot.input;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.command.DefaultableSelectCommand;
import frc.lib.mode.ModeState;
import frc.robot.modes.AlgaeArmMode;
import frc.robot.modes.AlgaeWheelMode;
import frc.robot.modes.CoralArmMode;
import frc.robot.modes.CoralWheelMode;
import frc.robot.modes.DriveMode;
import frc.robot.modes.ElevatorMode;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeWheel;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralWheel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class DebugInputProcessor {
    // controllers
    private final CommandXboxController driver;

    // modes
    @Logged private final ModeState<DriveMode> swerveState;
    @Logged private final ModeState<ElevatorMode> elevatorState;

    @Logged private final ModeState<CoralArmMode> coralArmState;
    @Logged private final ModeState<CoralWheelMode> coralWheelState;

    @Logged private final ModeState<AlgaeArmMode> algaeArmState;
    @Logged private final ModeState<AlgaeWheelMode> algaeWheelState;

    // TODO implement a more elegant solution that avoids the possibility of overlapping mode switches, as that's currently undefined behavior
    /** the state that was most recently switched to, e.g., the state that controls how default commands behave */
    @Logged private ModeState<?> activeState;
    
    // processors
    private final SwerveInputProcessor swerveProcessor;
    private final ElevatorInputProcessor elevatorProcessor;

    @Logged private final CoralArmInputProcessor coralArmProcessor;
    @Logged private final CoralWheelInputProcessor coralWheelProcessor;

    @Logged private final AlgaeArmInputProcessor algaeArmProcessor;
    @Logged private final AlgaeWheelInputProcessor algaeWheelProcessor;

    public DebugInputProcessor(
        final Swerve swerve,
        final Elevator elevator,

        final CoralArm coralArm,
        final CoralWheel coralWheel,

        final AlgaeArm algaeArm,
        final AlgaeWheel algaeWheel,

        final CommandXboxController driver
    ) {
        // properties
        // - controllers
        this.driver = driver;

        // - states
        this.swerveState = new ModeState<>(DriveMode.ALIGN);
        this.elevatorState = new ModeState<>(ElevatorMode.MANUAL);

        this.coralArmState = new ModeState<>(CoralArmMode.MANUAL);
        this.coralWheelState = new ModeState<>(CoralWheelMode.MANUAL);
        
        this.algaeArmState = new ModeState<>(AlgaeArmMode.MANUAL);
        this.algaeWheelState = new ModeState<>(AlgaeWheelMode.MANUAL);

        this.activeState = swerveState;

        // - processors
        this.swerveProcessor = new SwerveInputProcessor(swerve, driver, swerveState);
        this.elevatorProcessor = new ElevatorInputProcessor(elevator, driver, elevatorState);

        this.coralArmProcessor = new CoralArmInputProcessor(coralArm, driver, coralArmState);
        this.coralWheelProcessor = new CoralWheelInputProcessor(coralWheel, driver, coralWheelState);

        this.algaeArmProcessor = new AlgaeArmInputProcessor(algaeArm, driver, algaeArmState);
        this.algaeWheelProcessor = new AlgaeWheelInputProcessor(algaeWheel, driver, algaeWheelState);
    }

    // configuration
    /** perform all configuration */
    public void configure() {
        configureSwitches();
        configureTriggers();
        configureDefaults();
    }

    /** configure mode-switching */
    public void configureSwitches() {
        // swerve state
        // d-pad up: a is swerve, and b is align
        swerveState.registerSwitch(this::setActiveState, DriveMode.SWERVE, driver.povUp().and(driver.a()));
        swerveState.registerSwitch(this::setActiveState, DriveMode.ALIGN, driver.povUp().and(driver.b()));
        
        // d-pad right: y is FL, b is FR, x is BL, a is BR, so normal directions but rotated 45 degrees CW
        swerveState.registerSwitch(this::setActiveState, DriveMode.FL_ONLY, driver.povRight().and(driver.y()));
        swerveState.registerSwitch(this::setActiveState, DriveMode.FR_ONLY, driver.povRight().and(driver.b()));
        swerveState.registerSwitch(this::setActiveState, DriveMode.BL_ONLY, driver.povRight().and(driver.x()));
        swerveState.registerSwitch(this::setActiveState, DriveMode.BR_ONLY, driver.povRight().and(driver.a()));

        // elevator state
        // d-pad left: a is manual
        elevatorState.registerSwitch(this::setActiveState, ElevatorMode.MANUAL, driver.povLeft().and(driver.a()));

        // coral and algae arm and wheel
        // d-pad down: a is manual coral arm, b is manual coral wheel, x is manual algae arm, y is manual algae wheel,
        //             left bumper is manual algae left wheel, right bumper is manual algae right wheel

        // coral arm state
        coralArmState.registerSwitch(this::setActiveState, CoralArmMode.MANUAL, driver.povDown().and(driver.a()));

        // coral wheel state
        coralWheelState.registerSwitch(this::setActiveState, CoralWheelMode.MANUAL, driver.povDown().and(driver.b()));

        // algae arm state
        algaeArmState.registerSwitch(this::setActiveState, AlgaeArmMode.MANUAL, driver.povDown().and(driver.x()));

        // algae wheel state
        algaeWheelState.registerSwitch(this::setActiveState, AlgaeWheelMode.MANUAL, driver.povDown().and(driver.y()));
        algaeWheelState.registerSwitch(this::setActiveState, AlgaeWheelMode.MANUAL_LEFT_ONLY, driver.povDown().and(driver.leftBumper()));
        algaeWheelState.registerSwitch(this::setActiveState, AlgaeWheelMode.MANUAL_RIGHT_ONLY, driver.povDown().and(driver.rightBumper()));
    }

    /** configure trigger-based commands */
    public void configureTriggers() {
        swerveProcessor.configureTriggers(this::isModeActive);
        elevatorProcessor.configureTriggers(this::isModeActive);

        coralArmProcessor.configureTriggers(this::isModeActive);
        coralWheelProcessor.configureTriggers(this::isModeActive);

        algaeArmProcessor.configureTriggers(this::isModeActive);
        algaeWheelProcessor.configureTriggers(this::isModeActive);
    }

    /** configure default commands */
    public void configureDefaults() {
        // initialize map
        Map<Subsystem, Map<ModeState<?>, Command>> defaults = new HashMap<>();

        // add commands to map
        swerveProcessor.configureDefaults(defaults);
        elevatorProcessor.configureDefaults(defaults);

        coralArmProcessor.configureDefaults(defaults);
        coralWheelProcessor.configureDefaults(defaults);

        algaeArmProcessor.configureDefaults(defaults);
        algaeWheelProcessor.configureDefaults(defaults);

        // set per-mode active-state-dependent default commands
        for (Map.Entry<Subsystem, Map<ModeState<?>, Command>> entry : defaults.entrySet()) {
            Subsystem subsystem = entry.getKey();
            Map<ModeState<?>, Command> commands = entry.getValue();

            subsystem.setDefaultCommand(new DefaultableSelectCommand<>(commands, this::getActiveState));
        }
    }

    // properties
    public ModeState<?> getActiveState() {
        return this.activeState;
    }

    private void setActiveState(ModeState<?> state) {
        this.activeState = state;
    }

    // triggers
    public BooleanSupplier isModeActive(ModeState<?> state) {
        return () -> (state == getActiveState());
    }
}
