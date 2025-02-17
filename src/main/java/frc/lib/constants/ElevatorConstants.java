package frc.lib.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ElevatorConstants {
    // CAN ids via Rev Hardware Client
    public final static int
        LEFT_MOTOR = 0,
        RIGHT_MOTOR = 0;
    
    // PID and FF constants
    public final static double
        KP = 0.0,
        KI = 0.0,
        KD = 0.0,

        KG = 0.0,
        KS = 0.0,
        KV = 0.0,
        KA = 0.0;
    
    // TODO measure the necessary values to calculate this and split it into multiple variables, if necessary
    /** conversion factor between a single rotation of an elevator motor and the vertical meters the elevator travels */
    public final static double ELEVATOR_CF = 0.0;

    // TODO measure and determine these
    // TODO decide whether zero height will be defined as the floor or the lowest height the elevator can reach
    public final static Distance MAX_HEIGHT = Meters.of(0.0);
    public final static LinearVelocity MAX_SPEED = MetersPerSecond.of(0.0);
    public final static LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(0.0);

    // TODO measure these
    public final static Distance
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
