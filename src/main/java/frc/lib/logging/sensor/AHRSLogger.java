package frc.lib.logging.sensor;

import com.studica.frc.AHRS;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(AHRS.class)
public class AHRSLogger extends ClassSpecificLogger<AHRS> {
    public AHRSLogger() {
        super(AHRS.class);
    }

    @Override
    public void update(EpilogueBackend backend, AHRS ahrs) {
        backend.log("Continuous yaw (degrees)", ahrs.getAngle());
        backend.log("Yaw (degrees)", ahrs.getYaw());
        backend.log("Heading (degrees)", ahrs.getFusedHeading());
    }
}
