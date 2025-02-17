package frc.lib.coordinates.translations;

import java.util.function.Function;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.coordinates.Orientation;
import frc.lib.utility.Translation3dUtils;

/**
 * a 3-dimensional velocity in a specific coordinate system with a specific orientation using specific units
 */
public final class Velocity3 extends Translation3<LinearVelocityUnit> {
    // initialization
    protected Velocity3(CoordinateSystem system, Orientation orientation, LinearVelocityUnit unit, Translation3d position) {
        super(system, orientation, unit, position);
    }

    protected Velocity3(Velocity3 trans) {
        super(trans);
    }

    public Velocity3(CoordinateSystem system, Orientation orientation, LinearVelocityUnit unit, LinearVelocity x, LinearVelocity y, LinearVelocity z) {
        super(system, orientation, unit, x, y, z);
    }

    // property switching
    public Velocity3 system(CoordinateSystem newSystem) {
        if (system.equals(newSystem)) return new Velocity3(this);

        return new Velocity3(newSystem, orientation, unit, CoordinateSystem.convert(position, system, newSystem));
    }

    public Velocity3 orientation(Orientation newOrientation, AHRS navX2) {
        if (orientation == newOrientation) return new Velocity3(this);

        return new Velocity3(system, newOrientation, unit, Orientation.convert(position, system, orientation, newOrientation, navX2));
    }

    public Velocity3 unit(LinearVelocityUnit newUnit) {
        if (unit.equals(newUnit)) return new Velocity3(this);

        return new Velocity3(system, orientation, newUnit, Translation3dUtils.convertUnits(position, unit, newUnit));
    }

    // unit type switching
    public Acceleration3 per(Time time) {
        return new Acceleration3(system, orientation, unit.per(time.unit()), x().div(time), y().div(time), z().div(time));
    }

    public <Z extends ZeroPosition> Distance3<Z> times(Time time) {
        return new Distance3<>(system, orientation, unit.mult(time.unit()), x().times(time), y().times(time), z().times(time));
    }

    // operations
    public Velocity3 plus(Velocity3 other, AHRS navX2) {
        Translation3d inThisOrientation = Orientation.convert(other.position, other.system, other.orientation, orientation, navX2);
        Translation3d inThisSystem = CoordinateSystem.convert(inThisOrientation, other.system, system);
        Translation3d inThisUnit = Translation3dUtils.convertUnits(inThisSystem, other.unit, unit);
        
        return new Velocity3(system, orientation, unit, position.plus(inThisUnit));
    }

    public Velocity3 minus(Velocity3 other, AHRS navX2) {
        Translation3d inThisOrientation = Orientation.convert(other.position, other.system, other.orientation, orientation, navX2);
        Translation3d inThisSystem = CoordinateSystem.convert(inThisOrientation, other.system, system);
        Translation3d inThisUnit = Translation3dUtils.convertUnits(inThisSystem, other.unit, unit);
        
        return new Velocity3(system, orientation, unit, position.minus(inThisUnit));
    }

    public Velocity3 times(double scalar) {
        return new Velocity3(system, orientation, unit, position.times(scalar));
    }

    public Velocity3 div(double scalar) {
        return new Velocity3(system, orientation, unit, position.div(scalar));
    }

    // components
    // - getters
    /** returns the x velocity */
    public LinearVelocity x() {
        return unit.of(position.getX());
    }

    /** returns the y velocity */
    public LinearVelocity y() {
        return unit.of(position.getY());
    }

    /** returns the z velocity */
    public LinearVelocity z() {
        return unit.of(position.getZ());
    }

    // - setters
    /** sets the x velocity */
    public Velocity3 x(LinearVelocity x) {
        return new Velocity3(system, orientation, unit, x, y(), z());
    }

    /** sets the y velocity */
    public Velocity3 y(LinearVelocity y) {
        return new Velocity3(system, orientation, unit, x(), y, z());
    }

    /** sets the z velocity */
    public Velocity3 z(LinearVelocity z) {
        return new Velocity3(system, orientation, unit, x(), y(), z);
    }

    // - appliers
    /** sets the x velocity to the return value of the function that takes the current x velocity */
    public Velocity3 applyX(Function<LinearVelocity, LinearVelocity> function) {
        return x(function.apply(x()));
    }

    /** sets the y velocity to the return value of the function that takes the current y velocity */
    public Velocity3 applyY(Function<LinearVelocity, LinearVelocity> function) {
        return y(function.apply(y()));
    }

    /** sets the z velocity to the return value of the function that takes the current z velocity */
    public Velocity3 applyZ(Function<LinearVelocity, LinearVelocity> function) {
        return z(function.apply(z()));
    }
}
