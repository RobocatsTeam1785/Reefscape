package frc.lib.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

/** constants that apply to the entire robot */
public class RobotConstants {
    /** whether the robot is being tuned - used for checking whether to automatically update PID and FF gains */
    public static final boolean TUNING = true;

    // TODO measure this value more precisely
    public static final Mass ROBOT_WEIGHT = Pounds.of(115.0);

    /** width of the robot, including the bumper */
    // TODO measure this
    public static final Distance ROBOT_WIDTH = Inches.of(0.0);

    /** length of the robot, including the bumper */
    // TODO measure this
    public static final Distance ROBOT_LENGTH = Inches.of(0.0);

    // TODO measure this value using sysid, as the current method assumes equal distribution of weight, which works, but isn't ideal
    public static final MomentOfInertia ROBOT_MOI;

    static {
        // calculated using formula listed on https://pathplanner.dev/robot-config.html#robot-config-options
        double massKg = ROBOT_WEIGHT.in(Kilograms);

        double widthMetersSquared = Math.pow(ROBOT_WIDTH.in(Meters), 2.0);
        double lengthMetersSquared = Math.pow(ROBOT_LENGTH.in(Meters), 2.0);

        double moiKgMetersSquared = (1 / 12.0) * massKg * (widthMetersSquared + lengthMetersSquared);

        ROBOT_MOI = KilogramSquareMeters.of(moiKgMetersSquared);
    }
}
