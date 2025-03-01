package frc.lib.coordinates.translations;

import java.util.function.Function;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Distance;
import frc.lib.coordinates.Orientation;
import frc.lib.utility.Translation3dUtils;

/**
 * a 3-dimensional distance in a specific coordinate system with a specific orientation using specific units with a specific origin
 */
public final class Distance3<Z extends ZeroPosition> extends Translation3<DistanceUnit> {
    // initialization
    public Distance3(CoordinateSystem system, Orientation orientation, DistanceUnit unit, Translation3d position) {
        super(system, orientation, unit, position);
    }

    public Distance3(CoordinateSystem system, Orientation orientation, DistanceUnit unit, Distance x, Distance y, Distance z) {
        super(system, orientation, unit, x, y, z);
    }

    public Distance3(Distance3<Z> trans) {
        super(trans);
    }

    // property switching
    public Distance3<Z> system(CoordinateSystem newSystem) {
        if (system.equals(newSystem)) return new Distance3<>(this);

        return new Distance3<>(newSystem, orientation, unit, CoordinateSystem.convert(position, system, newSystem));
    }

    public Distance3<Z> orientation(Orientation newOrientation, AHRS navX2) {
        if (orientation == newOrientation) return new Distance3<>(this);

        return new Distance3<>(system, newOrientation, unit, Orientation.convert(position, system, orientation, newOrientation, navX2));
    }

    public Distance3<Z> unit(DistanceUnit newUnit) {
        if (unit.equals(newUnit)) return new Distance3<>(this);

        return new Distance3<>(system, orientation, newUnit, Translation3dUtils.convertUnits(position, unit, newUnit));
    }

    // unit type switching
    public Velocity3 per(TimeUnit timeUnit) {
        return new Velocity3(system, orientation, unit.per(timeUnit), x().per(timeUnit), y().per(timeUnit), z().per(timeUnit));
    }

    // operations
    public Distance3<Z> plus(Distance3<Z> other, AHRS navX2) {
        Translation3d inThisOrientation = Orientation.convert(other.position, other.system, other.orientation, orientation, navX2);
        Translation3d inThisSystem = CoordinateSystem.convert(inThisOrientation, other.system, system);
        Translation3d inThisUnit = Translation3dUtils.convertUnits(inThisSystem, other.unit, unit);
        
        return new Distance3<>(system, orientation, unit, position.plus(inThisUnit));
    }

    public Distance3<Z> minus(Distance3<Z> other, AHRS navX2) {
        Translation3d inThisOrientation = Orientation.convert(other.position, other.system, other.orientation, orientation, navX2);
        Translation3d inThisSystem = CoordinateSystem.convert(inThisOrientation, other.system, system);
        Translation3d inThisUnit = Translation3dUtils.convertUnits(inThisSystem, other.unit, unit);
        
        return new Distance3<>(system, orientation, unit, position.minus(inThisUnit));
    }

    public Distance3<Z> times(double scalar) {
        return new Distance3<>(system, orientation, unit, position.times(scalar));
    }

    public Distance3<Z> div(double scalar) {
        return new Distance3<>(system, orientation, unit, position.div(scalar));
    }

    // components
    // - getters
    /** returns the x distance */
    public Distance x() {
        return unit.of(position.getX());
    }

    /** returns the y distance */
    public Distance y() {
        return unit.of(position.getY());
    }

    /** returns the z distance */
    public Distance z() {
        return unit.of(position.getZ());
    }

    // - setters
    /** sets the x distance */
    public Distance3<Z> x(Distance x) {
        return new Distance3<>(system, orientation, unit, x, y(), z());
    }

    /** sets the y distance */
    public Distance3<Z> y(Distance y) {
        return new Distance3<>(system, orientation, unit, x(), y, z());
    }

    /** sets the z distance */
    public Distance3<Z> z(Distance z) {
        return new Distance3<>(system, orientation, unit, x(), y(), z);
    }

    // - appliers
    /** sets the x distance to the return value of the function that takes the current x distance */
    public Distance3<Z> applyX(Function<Distance, Distance> function) {
        return x(function.apply(x()));
    }

    /** sets the y distance to the return value of the function that takes the current y distance */
    public Distance3<Z> applyY(Function<Distance, Distance> function) {
        return y(function.apply(y()));
    }

    /** sets the z distance to the return value of the function that takes the current z distance */
    public Distance3<Z> applyZ(Function<Distance, Distance> function) {
        return z(function.apply(z()));
    }
}
