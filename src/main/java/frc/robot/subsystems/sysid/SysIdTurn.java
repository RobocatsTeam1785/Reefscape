package frc.robot.subsystems.sysid;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drive;

/** drive subsystem to perform system identification for the turn feedforward and PID controller */
public class SysIdTurn extends Drive {
    private final SysIdRoutine routine;

    // initialization
    public SysIdTurn(double period) {
        // pass the period to the Drive constructor for discretizing ChassisSpeeds
        super(period);

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

    // test commands
    public Command quasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command dynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
