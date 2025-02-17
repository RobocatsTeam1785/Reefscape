package frc.lib.coordinates.translations;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import frc.lib.coordinates.Orientation;
import frc.lib.utility.Translation3dUtils;

/**
 * a 3-dimensional translation in a specific coordinate system with a specific orientation using specific units
 */
public sealed class Translation3<U extends Unit> permits Distance3, Velocity3, Acceleration3, PureTranslation3 {
    // properties
    protected final CoordinateSystem system;
    protected final Orientation orientation;
    protected final U unit;

    protected final Translation3d position;

    // initialization
    protected Translation3(CoordinateSystem system, Orientation orientation, U unit, Translation3d position) {
        this.system = system;
        this.orientation = orientation;
        this.position = position;
        this.unit = unit;
    }

    protected Translation3(Translation3<U> trans) {
        this(trans.system, trans.orientation, trans.unit, trans.position);
    }

    protected Translation3(CoordinateSystem system, Orientation orientation, U unit, Measure<U> x, Measure<U> y, Measure<U> z) {
        this(system, orientation, unit, new Translation3d(x.in(unit), y.in(unit), z.in(unit)));
    }

    // property switching
    protected Translation3<U> system(CoordinateSystem newSystem) {
        if (system.equals(newSystem)) return new Translation3<>(this);

        return new Translation3<>(newSystem, orientation, unit, CoordinateSystem.convert(position, system, newSystem));
    }

    protected Translation3<U> orientation(Orientation newOrientation, AHRS navX2) {
        if (orientation == newOrientation) return new Translation3<>(this);

        return new Translation3<>(system, newOrientation, unit, Orientation.convert(position, system, orientation, newOrientation, navX2));
    }

    protected Translation3<U> unit(U newUnit) {
        if (unit.equals(newUnit)) return new Translation3<>(this);

        return new Translation3<>(system, orientation, newUnit, Translation3dUtils.convertUnits(position, unit, newUnit));
    }

    // operations
    protected Translation3<U> plus(Translation3<U> other, AHRS navX2) {
        Translation3d inThisOrientation = Orientation.convert(other.position, other.system, other.orientation, orientation, navX2);
        Translation3d inThisSystem = CoordinateSystem.convert(inThisOrientation, other.system, system);
        Translation3d inThisUnit = Translation3dUtils.convertUnits(inThisSystem, other.unit, unit);
        
        return new Translation3<>(system, orientation, unit, position.plus(inThisUnit));
    }

    protected Translation3<U> minus(Translation3<U> other, AHRS navX2) {
        Translation3d inThisOrientation = Orientation.convert(other.position, other.system, other.orientation, orientation, navX2);
        Translation3d inThisSystem = CoordinateSystem.convert(inThisOrientation, other.system, system);
        Translation3d inThisUnit = Translation3dUtils.convertUnits(inThisSystem, other.unit, unit);
        
        return new Translation3<>(system, orientation, unit, position.minus(inThisUnit));
    }

    protected Translation3<U> times(double scalar) {
        return new Translation3<>(system, orientation, unit, position.times(scalar));
    }

    protected Translation3<U> div(double scalar) {
        return new Translation3<>(system, orientation, unit, position.div(scalar));
    }
}
