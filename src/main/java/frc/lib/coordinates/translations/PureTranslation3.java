package frc.lib.coordinates.translations;

import static edu.wpi.first.units.Units.Value;

import java.util.function.Function;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.measure.Dimensionless;
import frc.lib.coordinates.Orientation;
import frc.lib.utility.Translation3dUtils;

/**
 * a 3-dimensional pure translation in a specific coordinate system with a specific orientation using specific units
 */
public final class PureTranslation3 extends Translation3<DimensionlessUnit> {
    // initialization
    protected PureTranslation3(CoordinateSystem system, Orientation orientation, DimensionlessUnit unit, Translation3d position) {
        super(system, orientation, unit, position);
    }

    protected PureTranslation3(PureTranslation3 trans) {
        super(trans);
    }

    public PureTranslation3(CoordinateSystem system, Orientation orientation, DimensionlessUnit unit, Dimensionless x, Dimensionless y, Dimensionless z) {
        super(system, orientation, unit, x, y, z);
    }

    public PureTranslation3(CoordinateSystem system, Orientation orientation, double x, double y, double z) {
        super(system, orientation, Value, Value.of(x), Value.of(y), Value.of(z));
    }

    // property switching
    public PureTranslation3 system(CoordinateSystem newSystem) {
        if (system.equals(newSystem)) return new PureTranslation3(this);

        return new PureTranslation3(newSystem, orientation, unit, CoordinateSystem.convert(position, system, newSystem));
    }

    public PureTranslation3 orientation(Orientation newOrientation, AHRS navX2) {
        if (orientation == newOrientation) return new PureTranslation3(this);

        return new PureTranslation3(system, newOrientation, unit, Orientation.convert(position, system, orientation, newOrientation, navX2));
    }

    public PureTranslation3 unit(DimensionlessUnit newUnit) {
        if (unit.equals(newUnit)) return new PureTranslation3(this);

        return new PureTranslation3(system, orientation, newUnit, Translation3dUtils.convertUnits(position, unit, newUnit));
    }

    // operations
    public PureTranslation3 plus(PureTranslation3 other, AHRS navX2) {
        Translation3d inThisOrientation = Orientation.convert(other.position, other.system, other.orientation, orientation, navX2);
        Translation3d inThisSystem = CoordinateSystem.convert(inThisOrientation, other.system, system);
        Translation3d inThisUnit = Translation3dUtils.convertUnits(inThisSystem, other.unit, unit);
        
        return new PureTranslation3(system, orientation, unit, position.plus(inThisUnit));
    }

    public PureTranslation3 minus(PureTranslation3 other, AHRS navX2) {
        Translation3d inThisOrientation = Orientation.convert(other.position, other.system, other.orientation, orientation, navX2);
        Translation3d inThisSystem = CoordinateSystem.convert(inThisOrientation, other.system, system);
        Translation3d inThisUnit = Translation3dUtils.convertUnits(inThisSystem, other.unit, unit);
        
        return new PureTranslation3(system, orientation, unit, position.minus(inThisUnit));
    }

    public PureTranslation3 times(double scalar) {
        return new PureTranslation3(system, orientation, unit, position.times(scalar));
    }

    public PureTranslation3 div(double scalar) {
        return new PureTranslation3(system, orientation, unit, position.div(scalar));
    }

    // components
    // - getters
    /** returns the x */
    public Dimensionless x() {
        return unit.of(position.getX());
    }

    /** returns the y */
    public Dimensionless y() {
        return unit.of(position.getY());
    }

    /** returns the z */
    public Dimensionless z() {
        return unit.of(position.getZ());
    }

    // - setters
    /** sets the x */
    public PureTranslation3 x(Dimensionless x) {
        return new PureTranslation3(system, orientation, unit, x, y(), z());
    }

    /** sets the y */
    public PureTranslation3 y(Dimensionless y) {
        return new PureTranslation3(system, orientation, unit, x(), y, z());
    }

    /** sets the z */
    public PureTranslation3 z(Dimensionless z) {
        return new PureTranslation3(system, orientation, unit, x(), y(), z);
    }

    // - appliers
    /** sets the x to the return value of the function that takes the current x */
    public PureTranslation3 applyX(Function<Dimensionless, Dimensionless> function) {
        return x(function.apply(x()));
    }

    /** sets the y to the return value of the function that takes the current y */
    public PureTranslation3 applyY(Function<Dimensionless, Dimensionless> function) {
        return y(function.apply(y()));
    }

    /** sets the z to the return value of the function that takes the current z */
    public PureTranslation3 applyZ(Function<Dimensionless, Dimensionless> function) {
        return z(function.apply(z()));
    }
}
