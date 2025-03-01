package frc.lib.coordinates.rotations;

import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Unit;
import frc.lib.coordinates.Orientation;

/**
 * a 3-dimensional rotation in a ...
 */
public class Rotation3<U extends Unit> {
    // properties
    // TODO convey how handedness is encoded in the coordinate system somehow
    public final CoordinateSystem system;
    public final Orientation orientation;

    public final U unit;

    // TODO switch to quaternions
    public final Rotation3d rotation;

    // initialization
    public Rotation3(CoordinateSystem system, Orientation orientation, U unit, Rotation3d rotation) {
        this.system = system;
        this.orientation = orientation;

        this.unit = unit;

        this.rotation = rotation;
    }
}
