package frc.lib.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.controller.PIDController;

@CustomLoggerFor(PIDController.class)
public class PIDControllerLogger extends ClassSpecificLogger<PIDController> {
    public PIDControllerLogger() {
        super(PIDController.class);
    }

    @Override
    public void update(EpilogueBackend backend, PIDController controller) {
        backend.log("Error (units)", controller.getError());
        backend.log("Error over time (units/s)", controller.getErrorDerivative());
        backend.log("Accumulated error (units)", controller.getAccumulatedError());
    }
}
