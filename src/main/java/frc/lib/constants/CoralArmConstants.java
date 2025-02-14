package frc.lib.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;

public class CoralArmConstants {
    // TODO tune these
    // PID and FF constants
    public final static double
        KP = 0.0,
        KI = 0.0,
        KD = 0.0,

        KS = 0.0,
        KG = 0.0,
        KV = 0.0,
        KA = 0.0;

    // TODO measure the necessary values to calculate this and split it into multiple variables, if necessary
    /** conversion factor between a single rotation of the arm motor and the rotation of the arm, in radians */
    public final static double ARM_CF = 1.0;

    // TODO measure and determine these
    // maximum speed and acceleration
    public final static AngularVelocity MAX_SPEED = RadiansPerSecond.of(1.0);
    public final static AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(1.0);

    // TODO measure these more specifically
    // range of motion
    public final static Angle MAX_ANGLE = Degrees.of(30);
    public final static Angle MIN_ANGLE = Degrees.of(-90);
}
