package frc.lib.mode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class ModeState<M extends Mode> {
    /** cache to store triggers comparing current and possible modes, to avoid creating an arbitrary number of redundant Triggers */
    private final HashMap<M, Trigger> isCache = new HashMap<>();

    /** list of mode switch triggers */
    private final ArrayList<Trigger> switches = new ArrayList<>();

    /** whether all mode switch triggers are inactive */
    private final Trigger noSwitchesActive = new Trigger(() -> switches.stream().allMatch(t -> !t.getAsBoolean()));

    /** the mode the controller uses at startup */
    public final M defaultMode;

    /** the current mode the controller is using */
    private M currentMode;

    public ModeState(M defaultMode) {
        this.defaultMode = defaultMode;
        currentMode = defaultMode;
    }

    // mode
    /** returns the current mode */
    public M mode() {
        return currentMode;
    }

    @Logged
    /** returns the name of the current mode */
    public String modeName() {
        return currentMode.name();
    }

    @Logged
    public String modeClass() {
        return currentMode.getClass().getName();
    }

    // switching
    /** registers a mode switch that sets the current mode to the specified mode when the trigger becomes true */
    public void registerSwitch(M mode, Trigger trigger) {
        // record the switch
        switches.add(trigger);

        // change the mode when the trigger becomes true
        trigger.onTrue(new InstantCommand(() -> {
            currentMode = mode;
        }));
    }

    /** registers a mode switch that sets the provided active state to this and the current mode to the specified mode when the trigger becomes true */
    public void registerSwitch(Consumer<ModeState<?>> setActiveState, M mode, Trigger trigger) {
        // record the switch
        switches.add(trigger);

        // change the mode and active state when the trigger becomes true
        trigger.onTrue(new InstantCommand(() -> {
            setActiveState.accept(this);
            currentMode = mode;
        }));
    }

    // triggers
    /** returns a trigger describing whether the current mode equals the specified mode */
    public Trigger is(M mode) {
        isCache.putIfAbsent(mode, new Trigger(() -> mode == mode()));
        return isCache.get(mode);
    }

    /** whether all mode switch triggers are inactive */
    public Trigger noSwitchesActive() {
        return noSwitchesActive;
    }

    // commands
    /** returns a command that selects between the given command map based on the current mode */
    public SelectCommand<M> select(Map<M, Command> commands) {
        return new SelectCommand<>(commands, this::mode);
    }

    /** returns a command that selects between the given runnable map based on the current mode, with each runnable having the specified requirements */
    public SelectCommand<M> selectRunnable(Map<M, Runnable> runnables, Subsystem... requirements) {
        // get the entries, convert the set to a stream, replace each runnable with an instant command that runs it,
        // and collect that stream of entries back into a map of modes and commands
        Map<M, Command> commands = runnables
            .entrySet()
            .stream()
            .map(e -> Map.entry(
                e.getKey(),
                new InstantCommand(e.getValue(), requirements)
            ))
            .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));
        
        return new SelectCommand<>(commands, this::mode);
    }
}
