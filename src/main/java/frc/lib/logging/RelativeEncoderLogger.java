package frc.lib.logging;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(RelativeEncoder.class)
public class RelativeEncoderLogger extends ClassSpecificLogger<RelativeEncoder> {
    public RelativeEncoderLogger() {
        super(RelativeEncoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, RelativeEncoder encoder) {
        backend.log("Position (units)", encoder.getPosition());
        backend.log("Absolute position (units)", Math.abs(encoder.getPosition()));

        backend.log("Velocity (units per second)", encoder.getVelocity());
        backend.log("Absolute velocity (units per second)", Math.abs(encoder.getVelocity()));
    }
}
