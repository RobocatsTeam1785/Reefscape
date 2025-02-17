package frc.lib.utility;

import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.CoordinateSystem;

public class CoordinateSystems {
    // TODO account for the fact that coordinate systems that switch handedness from NWU have a determinant of -1 and thus raise an error during initialization
    public static final CoordinateSystem
        // axis-based
        ENU = new CoordinateSystem(CoordinateAxis.E(), CoordinateAxis.N(), CoordinateAxis.U()),

        // aliases
        // source: the table on https://pdocs.kauailabs.com/navx-mxp/installation/orientation-2/
        NAVX = ENU;
}
