// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.command;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Map;
import java.util.function.Supplier;

/**
 * A command composition that runs one of a selection of commands using a selector and a key to
 * command mapping.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually, and the composition requires all
 * subsystems its components require.
 *
 * <p>This class is provided by the NewCommands VendorDep
 *
 * @param <K> The type of key used to select the command
 */
public class DefaultableSelectCommand<K> extends Command {
  public final Map<K, Command> m_commands;
  public final Supplier<? extends K> m_selector;
  public Command m_selectedCommand;
  public boolean m_runsWhenDisabled = true;
  public InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

  public final Command m_defaultCommand;

  /**
   * Creates a new SelectCommand.
   *
   * @param commands the map of commands to choose from
   * @param selector the selector to determine which command to run
   * @param defaultCommand the command to run when a selector that does not map to a command is present
   */
  @SuppressWarnings("this-escape")
  public DefaultableSelectCommand(Map<K, Command> commands, Supplier<? extends K> selector, Command defaultCommand) {
    m_commands = requireNonNullParam(commands, "commands", "SelectCommand");
    m_selector = requireNonNullParam(selector, "selector", "SelectCommand");
    m_defaultCommand = defaultCommand;

    CommandScheduler.getInstance().registerComposedCommands(m_defaultCommand);
    CommandScheduler.getInstance()
        .registerComposedCommands(commands.values().toArray(new Command[] {}));

    for (Command command : m_commands.values()) {
      addRequirements(command.getRequirements());
      m_runsWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        m_interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  /**
   * Creates a new SelectCommand that does nothing when the selector does not map to a command.
   *
   * @param commands the map of commands to choose from
   * @param selector the selector to determine which command to run
   */
  public DefaultableSelectCommand(Map<K, Command> commands, Supplier<? extends K> selector) {
    this(commands, selector, new InstantCommand());
  }

  @Override
  public void initialize() {
    m_selectedCommand = m_commands.getOrDefault(m_selector.get(), m_defaultCommand);
    m_selectedCommand.initialize();
  }

  @Override
  public void execute() {
    m_selectedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_selectedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_selectedCommand.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_runsWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_interruptBehavior;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addStringProperty(
        "selected", () -> m_selectedCommand == null ? "null" : m_selectedCommand.getName(), null);
  }
}
