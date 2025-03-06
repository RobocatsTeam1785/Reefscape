package frc.lib.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ElevatorConstants {
    // CAN ids via Rev Hardware Client
    // TODO decide on these
    public static final int
        LEFT_MOTOR_ID = 13,
        RIGHT_MOTOR_ID = 14;
    
    // TODO tune these
    // PID and FF constants
    public static final double
        KP = 0.2,
        KI = 0.0,
        KD = 0.0,

        KG = 0.38,
        KS = 0.72,
        KV = 0.1,
        KA = 0.0;
    
    // conversion factor
    // 20:1 gear ratio
    public static final double GEAR_RATIO = 20.0;

    /** diameter of the reel the motor rotates */
    public static final Distance REEL_DIAMETER = Inches.of(2.25);

    /** circumference of the reel */
    public static final Distance REEL_CIRCUMFERENCE = REEL_DIAMETER.times(Math.PI);

    /**
     * factor the carriage speed is multiplied by in comparison to the base pulley speed
     * <p>
     * equal to 3 because we have 4 total stages, and we gain an extra factor per stage
     */
    public static final double CARRIAGE_SPEED_SCALE = 3.0;

    /**
     * conversion factor between a single rotation of an elevator motor and the vertical meters the elevator travels
     * 
     * <ol>
     *   <li>divide by gear ratio, as the ratio of output rotations to input rotations is the inverse of the gear ratio</li>
     *   <li>multiply by reel circumference, as the elevator height equals the distance the reel travels, not the number of rotations</li>
     *   <li>multiply by the carriage speed scale to account for the speedup caused by changing stages three times, each increasing the speed</li>
     * </ol>
     */
    public static final double ELEVATOR_CF = 1.0 / GEAR_RATIO * REEL_CIRCUMFERENCE.in(Meters) * CARRIAGE_SPEED_SCALE;

    // voltage limitations
    public static final double SPEED_DEADBAND = 0.1;

    // TODO measure and determine these
    // TODO decide whether zero height will be defined as the floor or the lowest height the elevator can reach
    public static final Distance MAX_HEIGHT = Meters.of(1.14);
    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(0.75);
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(0.5);

    public static final Distance HEIGHT_SOFT_LIMIT = MAX_HEIGHT.minus(Inches.of(1.0));

    // scoring/intake-relevant heights
    // TODO measure these
    public static final Distance
        GROUND = Meters.of(0.0),

        // coral intake and score height
        CORAL_STATION_INTAKE_HEIGHT = Meters.of(0.0),

        REEF_L1_CORAL_SCORE_HEIGHT = Meters.of(0.0),
        REEF_L2_CORAL_SCORE_HEIGHT = Meters.of(0.0),
        REEF_L3_CORAL_SCORE_HEIGHT = Meters.of(0.0),
        REEF_L4_CORAL_SCORE_HEIGHT = Meters.of(0.0),

        // algae intake and score height
        REEF_L2_ALGAE_INTAKE_HEIGHT = Meters.of(0.0),
        REEF_L3_ALGAE_INTAKE_HEIGHT = Meters.of(0.0),

        PROCESSOR_ALGAE_SCORE_HEIGHT = Meters.of(0.0);
}
