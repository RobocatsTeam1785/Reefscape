package frc.lib.logging.motor;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
    public TalonFXLogger() {
        super(TalonFX.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFX controller) {
        double position = controller.getPosition().getValue().in(Radians);
        double velocity = controller.getVelocity().getValue().in(RadiansPerSecond);
        double acceleration = controller.getAcceleration().getValue().in(RadiansPerSecondPerSecond);

        double absPosition = Math.abs(position);
        double absVelocity = Math.abs(velocity);
        double absAcceleration = Math.abs(acceleration);

        backend.log("Position (radians)", position);
        backend.log("Velocity (radians/s)", velocity);
        backend.log("Acceleration (radians/s^2)", acceleration);

        backend.log("Absolute Position (radians)", absPosition);
        backend.log("Absolute Velocity (radians/s)", absVelocity);
        backend.log("Absolute Acceleration (radians/s^2)", absAcceleration);
    }
}
