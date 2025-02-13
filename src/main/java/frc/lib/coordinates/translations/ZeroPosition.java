package frc.lib.coordinates.translations;

/**
 * a distinct type of origin point that must be explicitly converted to others, if possible
 * <br></br>
 * no conversion methods exist as you cannot predict with certainty where you would be using a different zero point without perfect odometry
 * <br></br>
 * rather, field-relative should be estimated using sensor data, and robot-relative coordinates should be measured directly or calculated
 */
public class ZeroPosition {
    /**
     * robot-relative coordinates - defined as (0, 0, 0) when at the center of the robot at floor height
     */
    public class RobotCenter extends ZeroPosition {}

    /**
     * field-relative coordinates - defined as (0, 0, 0) when at the leftmost corner of the blue alliance wall at floor height,
     * when standing in the center of the field facing it
     */
    public class BlueCorner extends ZeroPosition {}
}
