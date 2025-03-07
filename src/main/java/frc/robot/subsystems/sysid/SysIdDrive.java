package frc.robot.subsystems.sysid;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Swerve;

/** drive subsystem to perform system identification for the drive feedforward and PID controller */
public class SysIdDrive extends Swerve {
    public final SysIdRoutine routine;

    // initialization
    public SysIdDrive(double period) {
        // pass the period to the Drive constructor for discretizing ChassisSpeeds
        super(period, pose -> Optional.empty());

        // 1 V/s ramp rate in a quasistatic test, 0V to 7V acceleration in a dynamic test, 10s timeout, and no external recorder
        SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10), null);
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(voltage -> {
            for (int i = 0; i < 4; i++) {
                modules[i].sysIdTurn(voltage);
            }
        }, log -> {
            for (int i = 0; i < 4; i++) {
                modules[i].sysIdTurnLog(log);
            }
        }, this);

        routine = new SysIdRoutine(config, mechanism);
    }

    // setup commands
    public Command alignModules(double yawRadians) {
        return run(() -> {
            // set the setpoint's angle to be the same as the robot's direction, so all motors move forward
            SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(yawRadians));

            for (int i = 0; i < 4; i++) {
                modules[i].updateSetpoint(state, i);
            }
        }).until(() -> {
            double totalErrorRadians = 0.0;

            for (int i = 0; i < 4; i++) {
                totalErrorRadians += modules[i].turnErrorRadians();
            }

            return totalErrorRadians < 0.1;
        });
    }

    // test commands
    public Command quasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction).beforeStarting(alignModules(navX2.getAngle()));
    }

    public Command dynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction).beforeStarting(alignModules(navX2.getAngle()));
    }
}
