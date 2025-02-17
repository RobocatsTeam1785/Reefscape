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

        // apply modulo
        double constrainedRadians = angle.in(Radians) % (2 * Math.PI);

        // modulo alone produces both a negative and positive region, so we need to move the negative region into the positive
        if (constrainedRadians < 0) constrainedRadians += (2 * Math.PI);

        backend.log("Radians (modulo)", constrainedRadians);
    }
}
