package frc.lib.logging.sensor;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(CANcoder.class)
public class CANCoderLogger extends ClassSpecificLogger<CANcoder> {
    public CANCoderLogger() {
        super(CANcoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANcoder encoder) {
        double position = encoder.getPosition().getValue().in(Radians);
        double velocity = encoder.getVelocity().getValue().in(RadiansPerSecond);

        double absolutePosition = encoder.getAbsolutePosition().getValue().in(Radians);

        backend.log("Position (radians)", position);
        backend.log("Velocity (radians/s)", velocity);
        backend.log("Absolute Position (radians)", absolutePosition);
    }
}
