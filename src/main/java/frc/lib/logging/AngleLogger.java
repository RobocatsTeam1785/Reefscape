package frc.lib.logging;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.measure.Angle;

@CustomLoggerFor(Angle.class)
public class AngleLogger extends ClassSpecificLogger<Angle> {
    public AngleLogger() {
        super(Angle.class);
    }

    @Override
    public void update(EpilogueBackend backend, Angle angle) {
        backend.log("Radians", angle.in(Radians));
    }
}
