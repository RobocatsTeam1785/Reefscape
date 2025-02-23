package frc.lib.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

@CustomLoggerFor(DutyCycleEncoder.class)
public class DutyCycleEncoderLogger extends ClassSpecificLogger<DutyCycleEncoder> {
    public DutyCycleEncoderLogger() {
        super(DutyCycleEncoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, DutyCycleEncoder encoder) {
        backend.log("Value", encoder.get());
    }
}
