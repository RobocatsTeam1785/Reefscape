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
    protected final CoordinateSystem system;
    protected final Orientation orientation;

    protected final U unit;

    // TODO switch to quaternions
    protected final Rotation3d rotation;

    // initialization
    protected Rotation3(CoordinateSystem system, Orientation orientation, U unit, Rotation3d rotation) {
        this.system = system;
        this.orientation = orientation;

        this.unit = unit;

        this.rotation = rotation;
    }
}
