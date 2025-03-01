package frc.lib.coordinates.rotations;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;

/**
 * which rotational point is defined as zero
 */
public class ZeroDirection {
    // properties
    /** the amount of CW-positive (when looking in the same direction as any positive axis) angular offset
     * required to make the rotation about the axis in this system equal to zero in the base system */
    public final Angle offset;

    // initialization
    public ZeroDirection(Angle offset) {
        this.offset = offset;
    }

    // instances
    public static final ZeroDirection
        /** the base system; right of each axis when facing the same direction as each positive axis */
        RIGHT = new ZeroDirection(Radians.of(0));
    
    // conversion
    public static <U extends Unit> Rotation3d convert(Rotation3d rotation, U unit) {
        // TODO implement 
        throw new RuntimeException();
    }
}
