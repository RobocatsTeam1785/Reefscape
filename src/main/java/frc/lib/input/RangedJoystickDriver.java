package frc.lib.input;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RangedJoystickDriver {
    /** if joystickDeadband is x, then controller joystick values in the range [-x, x] get reduced to zero */
    public final double joystickDeadband;

    public final double buttonPositionReset;
    public final double buttonVelocityReset;
    public final double buttonVelocityResetVolts;

    /** the consumer that sets the mechanism's position setpoint to the specified value. units are provided by the user of this class. */
    public final DoubleConsumer positionConsumer;

    /** the consumer that sets the mechanism's velocity setpoint to the specified value. units are provided by the user of this class. */
    public final DoubleConsumer velocityConsumer;

    /** the consumer that directly sets the mechanism's voltage, in volts */
    public final DoubleConsumer voltageConsumer;

    /** whether to directly control voltage using the velocity value and range, if true, or control the velocity setpoint in radians per second, if false */
    public boolean directlyControlVoltage;

    public double buttonPosition;

    public double joystickPositionMinimum;
    public double joystickPositionMaximum;

    public double buttonVelocity;

    public double joystickVelocityMinimum;
    public double joystickVelocityMaximum;

    public RangedJoystickDriver(
        final double joystickDeadband, 

        final double buttonPositionReset,
        final double buttonVelocityReset,
        final double buttonVelocityResetVolts,

        final DoubleConsumer positionConsumer, 
        final DoubleConsumer velocityConsumer, 
        final DoubleConsumer voltageConsumer,

        boolean defaultDirectlyControlVoltage,

        double defaultButtonPosition,
        double defaultJoystickPositionMinimum,
        double defaultJoystickPositionMaximum,

        double defaultButtonVelocity,
        double defaultJoystickVelocityMinimum,
        double defaultJoystickVelocityMaximum
    ) {
        this.joystickDeadband = joystickDeadband;

        this.buttonPositionReset = buttonPositionReset;
        this.buttonVelocityResetVolts = buttonVelocityResetVolts;
        this.buttonVelocityReset = buttonVelocityReset;

        this.positionConsumer = positionConsumer;
        this.velocityConsumer = velocityConsumer;
        this.voltageConsumer = voltageConsumer;

        this.directlyControlVoltage = defaultDirectlyControlVoltage;

        this.buttonPosition = defaultButtonPosition;

        this.joystickPositionMinimum = defaultJoystickPositionMinimum;
        this.joystickPositionMaximum = defaultJoystickPositionMaximum;

        this.buttonVelocity = defaultButtonVelocity;

        this.joystickVelocityMinimum = defaultJoystickVelocityMinimum;
        this.joystickVelocityMaximum = defaultJoystickVelocityMaximum;
    }

    public RangedJoystickDriver(
        final double joystickDeadband, 

        final double buttonPositionReset, 
        final double buttonVelocityReset,
        final double buttonVelocityResetVolts, 

        final DoubleConsumer positionConsumer, 
        final DoubleConsumer velocityConsumer, 
        final DoubleConsumer voltageConsumer
    ) {
        this(
            joystickDeadband,

            buttonPositionReset,
            buttonVelocityResetVolts,
            buttonVelocityReset,

            positionConsumer,
            velocityConsumer,
            voltageConsumer,

            false,

            0.0,
            0.0,
            0.0,

            0.0,
            0.0,
            0.0
        );
    }

    public void configureTriggers(
        final CommandXboxController driver,
        final Subsystem subsystem,

        BooleanSupplier isActive,
        Trigger noSwitchesActive,
        Trigger isRelevantMode
    ) {
        driver.x().and(noSwitchesActive).and(isActive).and(isRelevantMode).onTrue(new InstantCommand(() -> {
            positionConsumer.accept(buttonPosition);
        }, subsystem));

        driver.y().and(noSwitchesActive).and(isActive).and(isRelevantMode).onTrue(new InstantCommand(() -> {
            if (directlyControlVoltage) {
                voltageConsumer.accept(buttonVelocity);
            } else {
                velocityConsumer.accept(buttonVelocity);
            }
        }, subsystem));

        driver.a().and(noSwitchesActive).and(isActive).and(isRelevantMode).onTrue(new InstantCommand(() -> {
            positionConsumer.accept(buttonPositionReset);
        }, subsystem));

        driver.b().and(noSwitchesActive).and(isActive).and(isRelevantMode).onTrue(new InstantCommand(() -> {
            if (directlyControlVoltage) {
                voltageConsumer.accept(buttonVelocityResetVolts);
            } else {
                velocityConsumer.accept(buttonVelocityReset);
            }
        }, subsystem));
    }

    public void driveWithinJoystickRange(final CommandXboxController driver) {
        // - fully up means -1, which is unintuitive, so it requires inversion
        double position = -driver.getLeftY();
        double velocity = -driver.getRightY();

        // avoid sending very small voltages that cause accidental drift
        position = MathUtil.applyDeadband(position, joystickDeadband);
        velocity = MathUtil.applyDeadband(velocity, joystickDeadband);

        position = joystickPositionMinimum + position * (joystickPositionMaximum - joystickPositionMinimum);
        velocity = joystickVelocityMinimum + velocity * (joystickVelocityMaximum - joystickVelocityMinimum);

        // only apply in-range position setpoints when the left bumper is down
        if (driver.leftBumper().getAsBoolean()) {
            positionConsumer.accept(position);
        }

        // only apply in-range velocity setpoints when the right bumper is down
        if (driver.rightBumper().getAsBoolean()) {
            if (directlyControlVoltage) {
                voltageConsumer.accept(velocity);
            } else {
                velocityConsumer.accept(velocity);
            }
        }
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RangedJoystickDriver");
        
        // mutable values
        builder.addDoubleProperty("Button Position", () -> buttonPosition, value -> buttonPosition = value);
        builder.addDoubleProperty("Joystick Position Minimum", () -> joystickPositionMinimum, value -> joystickPositionMinimum = value);
        builder.addDoubleProperty("Joystick Position Maximum", () -> joystickPositionMaximum, value -> joystickPositionMaximum = value);
        
        builder.addDoubleProperty("Button Velocity", () -> buttonVelocity, value -> buttonVelocity = value);
        builder.addDoubleProperty("Joystick Velocity Minimum", () -> joystickVelocityMinimum, value -> joystickVelocityMinimum = value);
        builder.addDoubleProperty("Joystick Velocity Maximum", () -> joystickVelocityMaximum, value -> joystickVelocityMaximum = value);
        
        builder.addBooleanProperty("Directly Control Voltage", () -> directlyControlVoltage, value -> directlyControlVoltage = value);
        
        // constants
        builder.publishConstDouble("Joystick Deadband", joystickDeadband);
        
        builder.publishConstDouble("Button Position Reset", buttonPositionReset);
        builder.publishConstDouble("Button Velocity Reset", buttonVelocityReset);
        builder.publishConstDouble("Button Velocity Reset Volts", buttonVelocityResetVolts);
    }
}
