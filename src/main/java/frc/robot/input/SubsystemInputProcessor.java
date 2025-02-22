package frc.robot.input;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.mode.ModeState;
import frc.robot.modes.DriveMode;
import frc.robot.subsystems.Swerve;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class SubsystemInputProcessor {
    // controllers
    private final CommandXboxController driver;

    // modes
    @Logged private final ModeState<DriveMode> swerveState;

    // TODO implement a more elegant solution that avoids the possibility of overlapping mode switches, as that's currently undefined behavior
    /** the state that was most recently switched to, e.g., the state that controls how default commands behave */
    @Logged private ModeState<?> activeState;
    
    // processors
    private final SwerveInputProcessor swerveProcessor;

    public SubsystemInputProcessor(final Swerve swerve, final CommandXboxController driver) {
        // properties
        // - controllers
        this.driver = driver;

        // - states
        this.swerveState = new ModeState<>(DriveMode.ALIGN);

        this.activeState = swerveState;

        // - processors
        this.swerveProcessor = new SwerveInputProcessor(swerve, driver, swerveState);
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
    }

    /** configure trigger-based commands */
    public void configureTriggers() {
        swerveProcessor.configureTriggers();
    }

    /** configure default commands */
    public void configureDefaults() {
        // initialize map
        Map<Subsystem, Map<ModeState<?>, Command>> defaults = new HashMap<>();

        // add commands to map
        swerveProcessor.configureDefaults(defaults);

        // set per-mode active-state-dependent default commands
        for (Map.Entry<Subsystem, Map<ModeState<?>, Command>> entry : defaults.entrySet()) {
            Subsystem subsystem = entry.getKey();
            Map<ModeState<?>, Command> commands = entry.getValue();

            subsystem.setDefaultCommand(new SelectCommand<>(commands, this::getActiveState));
        }
    }

    // properties
    public ModeState<?> getActiveState() {
        return this.activeState;
    }

    private void setActiveState(ModeState<?> state) {
        this.activeState = state;
    }
}
