package frc.lib.coordinates.translations;

import java.util.function.Function;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;
import frc.lib.coordinates.Orientation;
import frc.lib.utility.Translation3dUtils;

/**
 * a 3-dimensional acceleration in a specific coordinate system with a specific orientation using specific units
 */
public final class Acceleration3 extends Translation3<LinearAccelerationUnit> {
    // initialization
    public Acceleration3(CoordinateSystem system, Orientation orientation, LinearAccelerationUnit unit, Translation3d position) {
        super(system, orientation, unit, position);
    }

    public Acceleration3(Acceleration3 trans) {
        super(trans);
    }

    public Acceleration3(CoordinateSystem system, Orientation orientation, LinearAccelerationUnit unit, LinearAcceleration x, LinearAcceleration y, LinearAcceleration z) {
        super(system, orientation, unit, x, y, z);
    }

    // property switching
    public Acceleration3 system(CoordinateSystem newSystem) {
        if (system.equals(newSystem)) return new Acceleration3(this);

        return new Acceleration3(newSystem, orientation, unit, CoordinateSystem.convert(position, system, newSystem));
    }

    public Acceleration3 orientation(Orientation newOrientation, AHRS navX2) {
        if (orientation == newOrientation) return new Acceleration3(this);

        return new Acceleration3(system, newOrientation, unit, Orientation.convert(position, system, orientation, newOrientation, navX2));
    }

    public Acceleration3 unit(LinearAccelerationUnit newUnit) {
        if (unit.equals(newUnit)) return new Acceleration3(this);

        return new Acceleration3(system, orientation, newUnit, Translation3dUtils.convertUnits(position, unit, newUnit));
    }

    // unit type switching
    public Velocity3 times(Time time) {
        return new Velocity3(system, orientation, unit.mult(time.unit()), x().times(time), y().times(time), z().times(time));
    }

    // operations
    public Acceleration3 plus(Acceleration3 other, AHRS navX2) {
        Translation3d inThisOrientation = Orientation.convert(other.position, other.system, other.orientation, orientation, navX2);
        Translation3d inThisSystem = CoordinateSystem.convert(inThisOrientation, other.system, system);
        Translation3d inThisUnit = Translation3dUtils.convertUnits(inThisSystem, other.unit, unit);
        
        return new Acceleration3(system, orientation, unit, position.plus(inThisUnit));
    }

    public Acceleration3 minus(Acceleration3 other, AHRS navX2) {
        Translation3d inThisOrientation = Orientation.convert(other.position, other.system, other.orientation, orientation, navX2);
        Translation3d inThisSystem = CoordinateSystem.convert(inThisOrientation, other.system, system);
        Translation3d inThisUnit = Translation3dUtils.convertUnits(inThisSystem, other.unit, unit);
        
        return new Acceleration3(system, orientation, unit, position.minus(inThisUnit));
    }

    public Acceleration3 times(double scalar) {
        return new Acceleration3(system, orientation, unit, position.times(scalar));
    }

    public Acceleration3 div(double scalar) {
        return new Acceleration3(system, orientation, unit, position.div(scalar));
    }

    // components
    // - getters
    /** returns the x acceleration */
    public LinearAcceleration x() {
        return unit.of(position.getX());
    }

    /** returns the y acceleration */
    public LinearAcceleration y() {
        return unit.of(position.getY());
    }

    /** returns the z acceleration */
    public LinearAcceleration z() {
        return unit.of(position.getZ());
    }

    // - setters
    /** sets the x acceleration */
    public Acceleration3 x(LinearAcceleration x) {
        return new Acceleration3(system, orientation, unit, x, y(), z());
    }

    /** sets the y acceleration */
    public Acceleration3 y(LinearAcceleration y) {
        return new Acceleration3(system, orientation, unit, x(), y, z());
    }

    /** sets the z acceleration */
    public Acceleration3 z(LinearAcceleration z) {
        return new Acceleration3(system, orientation, unit, x(), y(), z);
    }

    // - appliers
    /** sets the x acceleration to the return value of the function that takes the current x acceleration */
    public Acceleration3 applyX(Function<LinearAcceleration, LinearAcceleration> function) {
        return x(function.apply(x()));
    }

    /** sets the y acceleration to the return value of the function that takes the current y acceleration */
    public Acceleration3 applyY(Function<LinearAcceleration, LinearAcceleration> function) {
        return y(function.apply(y()));
    }

    /** sets the z acceleration to the return value of the function that takes the current z acceleration */
    public Acceleration3 applyZ(Function<LinearAcceleration, LinearAcceleration> function) {
        return z(function.apply(z()));
    }
}
