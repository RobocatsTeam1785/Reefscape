package frc.lib.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class AlgaeWheelConstants {
    // CAN ids via Rev Hardware Client
    // TODO decide on this
    public static final int
        LEFT_MOTOR_ID = 18,
        RIGHT_MOTOR_ID = 19;

    // TODO tune these
    // PID and FF constants
    public static final double
        KP = 0.1,
        KI = 0.0,
        KD = 0.0,

        KS = 0.0,
        KV = 1.0,
        KA = 0.0;

    // 1:1 gear ratio
    public static final double GEAR_RATIO = 1.0;

    /** diamter of the green coral wheel */
    public static final Distance WHEEL_DIAMETER = Inches.of(4);

    /** circumference of the wheel */
    public static final Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(Math.PI);

    /**
     * conversion factor between a single rotation of the arm wheel motor and one meter of rotation
     * 
     * <ol>
     *   <li>divide by gear ratio, as the ratio of output rotations to input rotations is the reciprocal of the gear ratio</li>
     *   <li>multiply by circumference to obtain the distance travelled, not rotations, as, effectively, 1 wheel circumference = 1 rotation</li>
     * </ol>
     */
    public static final double WHEEL_CF = 1.0 / GEAR_RATIO * WHEEL_CIRCUMFERENCE.in(Meters);

    // TODO measure and determine these
    // maximum speed and acceleration
    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(1.0);
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(1.0);
}
