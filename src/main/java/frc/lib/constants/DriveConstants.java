package frc.lib.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
    /** whether to optimize swerve module states, e.g., reversing the motor speed instead of rotating 180 degrees and applying normal speed - used for testing when having the same sign is desirable */
    public static final boolean OPTIMIZE_STATES = true;

    // CAN ids - set using Rev Hardware Client and Phoenix Tuner X
    public static final int
        FL_ENCODER_ID = 1,
        FL_DRIVE_ID = 2,
        FL_TURN_ID = 3,

        FR_ENCODER_ID = 4,
        FR_DRIVE_ID = 5,
        FR_TURN_ID = 6,

        BL_ENCODER_ID = 7,
        BL_DRIVE_ID = 8,
        BL_TURN_ID = 9,

        BR_ENCODER_ID = 10,
        BR_DRIVE_ID = 11,
        BR_TURN_ID = 12;

    // feedforward and PID constants
    // TODO use sysid to calculate these constants
        public static final double
        TRANSLATIONAL_KP = 2.0,
        TRANSLATIONAL_KI = 0.0,
        TRANSLATIONAL_KD = 0.0,

        TRANSLATIONAL_KS = 0.5,
        TRANSLATIONAL_KV = 2.8,
        TRANSLATIONAL_KA = 0.1,

        // calculated using the log generated at 9:11 AM on 2/8/25
        // ROTATIONAL_KP = 43.539,
        // ROTATIONAL_KI = 0.0,
        // ROTATIONAL_KD = 12.983,

        // ROTATIONAL_KS = 0.20388,
        // ROTATIONAL_KV = 16.405,
        // ROTATIONAL_KA = 1.4434;

        // experimental values
        // ROTATIONAL_KP = 1.0,
        // ROTATIONAL_KI = 0.0,
        // ROTATIONAL_KD = 0.5,

        // ROTATIONAL_KS = 0.6,
        // ROTATIONAL_KV = 0.1,
        // ROTATIONAL_KA = 0.0;

        // experimental values
        // ROTATIONAL_KP = 2.0,
        // ROTATIONAL_KI = 0.0,
        // ROTATIONAL_KD = 0.0,

        // ROTATIONAL_KS = 0.4,
        // ROTATIONAL_KV = 1.0,
        // ROTATIONAL_KA = 1.0;

        // calculated using the log generated at 4:05 PM on 2/11/25
        ROTATIONAL_KP = 2.0, // 41.137,
        ROTATIONAL_KI = 0.0,
        ROTATIONAL_KD = 0.1, // 4.0337,

        ROTATIONAL_KS = 0.18342,
        ROTATIONAL_KV = 0.8, // 2.6252,
        ROTATIONAL_KA = 0*0.18483;//0.28483;
    
    // hardware constants - we're using the MK4i Swerve Module from Swerve Drive Specialties with the L2 gear ratio
    public static final double
        // both ratios are from https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module?variant=39598777270385
        // or, if that link dies, https://web.archive.org/web/20250208143202/https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module?variant=39598777270385

        // 150/7 : 1 - "The steering gear ratio of the MK4i is 150/7:1."
        TURN_GEAR_RATIO = 150.0 / 7.0,

        // 6.75 : 1 - in the table, in the "Overall Gear Ratio" row in the "L2 Ratio" column
        DRIVE_GEAR_RATIO = 6.75;

    // speed and acceleration constants
    // converts speed from [-x, x] to [-x, -0.1) U (0.1, x]
    public static final double TRANSLATIONAL_SPEED_DEADBAND = 0.1;
    public static final double ROTATIONAL_SPEED_DEADBAND = 0.1;

    // TODO test these 4 constants to ensure accuracy
    public static final LinearVelocity TRANSLATIONAL_MAX_SPEED = MetersPerSecond.of(2);
    public static final AngularVelocity ROTATIONAL_MAX_SPEED = RotationsPerSecond.of(0.5);

    public static final LinearAcceleration TRANSLATIONAL_MAX_ACCELERATION = MetersPerSecondPerSecond.of(3);
    public static final AngularAcceleration ROTATIONAL_MAX_ACCELERATION = RotationsPerSecondPerSecond.of(0.5);

    // position constants
    // distance from the robot's center to the center of a side, in meters
    // the robot is 29in wide, so the apothem is simply 29/2 in
    public static final Distance ROBOT_APOTHEM = Inches.of(29.0).div(2.0);

    public static final Distance WHEEL_RADIUS = Inches.of(2);
    public static final Distance WHEEL_CIRCUMFERENCE = WHEEL_RADIUS.times(2.0 * Math.PI);
    
    /* explanation of the WPILib coordinate system:
    *
    * positive X is forward, negative X is backwards
    * positive Y is left, negative Y is right
    * positive Z is up, negative Z is down
    * 
    * when a positive axis is facing you, CCW rotation about it is positive, and CW is negative
    * 
    * an angle of zero begins at the positive X axis (forwards), and increases as it moves CCW
    * 
    * (using https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html)
    */
    public static final Translation2d
        FL_POS = new Translation2d(ROBOT_APOTHEM, ROBOT_APOTHEM),
        FR_POS = new Translation2d(ROBOT_APOTHEM, ROBOT_APOTHEM.unaryMinus()),
        BL_POS = new Translation2d(ROBOT_APOTHEM.unaryMinus(), ROBOT_APOTHEM),
        BR_POS = new Translation2d(ROBOT_APOTHEM.unaryMinus(), ROBOT_APOTHEM.unaryMinus());
}
