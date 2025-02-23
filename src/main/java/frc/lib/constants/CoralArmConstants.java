package frc.lib.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;

public class CoralArmConstants {
    // CAN ids via Rev Hardware Client
    // TODO decide on this
    public static final int
        MOTOR_ID = 0;

    // digital channel, as physically configured in the RoboRIO DIO ports
    // TODO determine which one this is
    public static final int HEX_ENCODER_CHANNEL = 0;

    // TODO tune these
    // PID and FF constants
    public static final double
        KP = 0.0,
        KI = 0.0,
        KD = 0.0,

        KS = 0.0,
        KG = 0.0,
        KV = 0.0,
        KA = 0.0;

    // 20:1 gear ratio
    public static final double GEAR_RATIO = 20.0;

    /**
     * conversion factor between a single rotation of the arm motor and the rotation of the arm, in radians
     * 
     * <ol>
     *   <li>divide by gear ratio, as the ratio of output rotations to input rotations is the reciprocal of the gear ratio</li>
     *   <li>multiply by 2pi to convert to radians, as 2pi radians = 1 rotation</li>
     * </ol>
     */
    public static final double ARM_CF = 1.0 / GEAR_RATIO * (2 * Math.PI);

    // TODO measure and determine these
    // maximum speed and acceleration
    public static final AngularVelocity MAX_SPEED = RadiansPerSecond.of(1.0);
    public static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(1.0);

    // range of motion, where zero is defined as directly forward, and upwards rotation is defined as positive
    public static final Angle MAX_ANGLE = Degrees.of(55);
    public static final Angle MIN_ANGLE = Degrees.of(-90);
}
