package frc.lib.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.controller.ProfiledPIDController;

@CustomLoggerFor(ProfiledPIDController.class)
public class ProfiledPIDControllerLogger extends ClassSpecificLogger<ProfiledPIDController> {
    public ProfiledPIDControllerLogger() {
        super(ProfiledPIDController.class);
    }

    @Override
    public void update(EpilogueBackend backend, ProfiledPIDController controller) {
        backend.log("Error (units)", controller.getPositionError());
        backend.log("Error over time (units/s)", controller.getVelocityError());
        backend.log("Accumulated error (units)", controller.getAccumulatedError());

        backend.log("Setpoint position (units)", controller.getSetpoint().position);
        backend.log("Setpoint velocity (units/s)", controller.getSetpoint().velocity);
    }
}
