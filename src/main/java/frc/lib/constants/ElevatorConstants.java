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
    public static final int
        LEFT_MOTOR = 0,
        RIGHT_MOTOR = 0;
    
    // TODO tune these
    // PID and FF constants
    public static final double
        KP = 0.0,
        KI = 0.0,
        KD = 0.0,

        KG = 0.0,
        KS = 0.0,
        KV = 0.0,
        KA = 0.0;
    
    // conversion factor
    // 20:1 gear ratio
    public static final double GEAR_RATIO = 20.0;

    /** diameter of the reel the motor rotates */
    public static final Distance REEL_DIAMETER = Inches.of(2.25);

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
     *   <li>divide by gear ratio, as the ratio of input rotations to output rotations is the inverse of the gear ratio</li>
     *   <li>multiply by reel circumference, as the elevator height equals the distance the reel travels, not the number of rotations</li>
     *   <li>multiply by the carriage speed scale to account for the speedup caused by changing stages three times, each increasing the speed</li>
     * </ol>
     */
    public static final double ELEVATOR_CF = 1.0 / GEAR_RATIO * REEL_DIAMETER.in(Meters) * CARRIAGE_SPEED_SCALE;

    // TODO measure and determine these
    // TODO decide whether zero height will be defined as the floor or the lowest height the elevator can reach
    public static final Distance MAX_HEIGHT = Meters.of(0.0);
    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(0.2);
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(0.1);

    // TODO measure these
    public static final Distance
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
