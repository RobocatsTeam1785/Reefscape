package frc.robot.input;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.input.InputProcessor;
import frc.lib.mode.ModeState;
import frc.robot.modes.DriveMode;
import frc.robot.subsystems.Swerve;

public class SwerveInputProcessor extends InputProcessor {
    // subsystems
    private final Swerve swerve;

    // controllers
    private final CommandXboxController driver;

    // modes
    private final ModeState<DriveMode> state;

    // TODO make a read-only version of ModeState to disallow registering mode switches in an InputProcessor, outside of SubsystemInputProcessor
    public SwerveInputProcessor(final Swerve swerve, final CommandXboxController driver, final ModeState<DriveMode> state) {
        this.swerve = swerve;
        this.driver = driver;
        this.state = state;
    }

    @Override
    public void configureTriggers(Function<ModeState<?>, BooleanSupplier> isModeActive) {
        // defined suppliers
        BooleanSupplier isActive = isModeActive.apply(state);

        // ! all triggers must be logically ANDed with state.noSwitchesActive() and isActive in order to ensure that they
        // ! do not conflict with mode-switching triggers and only run when the current mode state is active
        // TODO ideally find a way to do this automatically

        // buttons
        // when a is pressed and in a single-module-only mode, zero the relative turn encoder
        // d-pad is negated to avoid collision with mode-switching
        // outside a single-module-only mode, literally rezero all relative turn encoders
        driver.a().and(state.noSwitchesActive()).and(isActive).onTrue(new InstantCommand(() -> {
            DriveMode mode = state.mode();

            if (mode.oneModuleOnly()) {
                swerve.zeroRelTurnEncoder(mode.id);
            } else {
                for (int i = 0; i < 4; i++) {
                    swerve.zeroRelTurnEncoder(i);
                }
            }
        }));

        // when b is pressed:
        //      in a single-module-only mode, set the turn encoder value
        //      to the absolute encoder value to rezero it
        //
        //      outside a single-module-only mode, rezero all turn encoders
        driver.b().and(state.noSwitchesActive()).and(isActive).onTrue(new InstantCommand(() -> {
            DriveMode mode = state.mode();

            if (mode.oneModuleOnly()) {
                swerve.recoverAbsAngle(mode.id);
            } else {
                swerve.recoverAbsAngles();
            }
        }));

        // state-based
        // zero turn voltage when the right trigger is lifted
        driver.rightTrigger(0.5).negate().and(state.noSwitchesActive()).and(isActive).onTrue(new InstantCommand(() -> {
            DriveMode mode = state.mode();

            if (mode.oneModuleOnly()) {
                swerve.zeroTurnVoltage(mode.id);
            } else if (mode == DriveMode.ALIGN) {
                swerve.zeroTurnVoltage();
            }
        }));
    }

    @Override
    public void configureDefaults(Map<Subsystem, Map<ModeState<?>, Command>> defaults) {
        Map<ModeState<?>, Command> commands = defaults.putIfAbsent(swerve, new HashMap<>());

        commands.put(state, state.selectRunnable(Map.of(
            DriveMode.SWERVE,  this::driveSwerve,
            DriveMode.ALIGN,   this::driveAlign,
            DriveMode.FL_ONLY, () -> driveOnly(0),
            DriveMode.FR_ONLY, () -> driveOnly(1),
            DriveMode.BL_ONLY, () -> driveOnly(2),
            DriveMode.BR_ONLY, () -> driveOnly(3)
        ), swerve));
    }

    // driving
    /** drives in swerve mode using the left joystick for translation and the right joystick for rotation */
    public void driveSwerve() {
        // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC
        // fully right means 1, which is intuitive
        double xSpeed = driver.getLeftX();

        // fully up means -1, which is unintuitive, so it requires inversion
        double ySpeed = -driver.getLeftY();

        // fully right means 1, which is positive; however, in WPILib, positive rotation means CCW rotation, and moving the joystick right is generally
        // associated with CW rotation, so it requires inversion
        double rotSpeed = -driver.getRightX();

        swerve.arcadeDrive(xSpeed, ySpeed, rotSpeed);
    }

    /**
     * aligns to the rotation designated by the driver left joystick, where fully right means zero, and counterclockwise is positive
     * in turn, "zero" is defined as the yaw the robot had when the navX was last zeroed
     * 
     * alignment only occurs when the right trigger is down
     */
    public void driveAlign() {
        if (driver.getRightTriggerAxis() <= 0.5) return;

        swerve.align(leftAlignAngle(driver));
    }

    /**
     * drives a single swerve module using the left joystick (up and down meaning forward and backward) and the right joystick to control angle,
     * where the id is in [0, 3], in the order FL, FR, BL, BR
     * 
     * alignment only occurs when the right trigger is down
     */
    public void driveOnly(int moduleId) {
        boolean alignEnabled = driver.getRightTriggerAxis() > 0.5;

        // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC
        // fully up means -1, which is unintuitive in ENU, so it requires inversion
        double speed = -driver.getLeftY();

        if (alignEnabled) {
            swerve.only(moduleId, speed, rightAlignAngle(driver));
        } else {
            swerve.only(moduleId, speed);
        }
    }
}
