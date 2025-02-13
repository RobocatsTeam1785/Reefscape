package frc.lib.coordinates;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.utility.CoordinateSystems;

/**
 * a distinct orientation that determines how the axes are rotated
 */
public abstract class Orientation {
    // TODO possibly implement rotation transformation

    // conversions
    protected abstract Translation3d toBase(Translation3d position, CoordinateSystem system, AHRS navX2);
    protected abstract Translation3d fromBase(Translation3d base, CoordinateSystem system, AHRS navX2);

    // implementations
    /**
     * the base orientation, in which:
     * <ul>
     *   <li>forward is defined as towards the USB/USB-C port of the navX2, when positioned inside the area of the navX2 board</li>
     *   <li>right is defined as towards the MXP connector, parallel with the longer side of the board</li>
     *   <li>up is defined as towards the sky, perpendicular to the board</li>
     * </ul>
     */
    public static final Orientation ROBOT_RELATIVE = new Orientation() {
        protected Translation3d toBase(Translation3d position, CoordinateSystem system, AHRS navX2) {
            return position;
        }

        protected Translation3d fromBase(Translation3d base, CoordinateSystem system, AHRS navX2) {
            return base;
        }
    };

    /**
     * the orientation in which:
     * <ul>
     *   <li>forward is defined as towards the center of the field, when standing by the blue alliance wall</li>
     *   <li>right is defined as rightwards, parallel with the blue alliance wall</li>
     *   <li>up is defined as towards the sky, perpendicular to the floor</li>
     * </ul>
     */
    public static final Orientation FIELD_RELATIVE = new Orientation() {
        protected Translation3d toBase(Translation3d position, CoordinateSystem system, AHRS navX2) {
            // calculate quaternion
            double w = navX2.getQuaternionW();
            double x = navX2.getQuaternionX();
            double y = navX2.getQuaternionY();
            double z = navX2.getQuaternionZ();
    
            // the orientation quaternion that describes the transformation from the orientation when the navX2 was last zeroed to the current orientation
            Quaternion zeroedToCurrent = new Quaternion(w, x, y, z);
    
            // switch to the navx coordinate system, rotate from the field-relative orientation (which should ALWAYS be equivalent to
            // the orientation when zeroed) to the robot-relative orientation, and switch back to the original coordinate system
            Translation3d inNavXSystem = CoordinateSystem.convert(position, system, CoordinateSystems.NAVX);
            Translation3d rotated = inNavXSystem.rotateBy(new Rotation3d(zeroedToCurrent));
            Translation3d inOriginalSystem = CoordinateSystem.convert(rotated, CoordinateSystems.NAVX, system);

            return inOriginalSystem;
        }

        protected Translation3d fromBase(Translation3d base, CoordinateSystem system, AHRS navX2) {
            // calculate quaternion
            double w = navX2.getQuaternionW();
            double x = navX2.getQuaternionX();
            double y = navX2.getQuaternionY();
            double z = navX2.getQuaternionZ();
    
            // the orientation quaternion that describes the transformation from the current orientation to the orientation when the navX2 was last zeroed
            Quaternion currentToZeroed = new Quaternion(w, x, y, z).inverse();
    
            // switch to the navx coordinate system, rotate from the robot-relative orientation to the field-relative orientation (which should ALWAYS be
            // equivalent to the orientation when zeroed), and switch back to the original coordinate system
            Translation3d inNavXSystem = CoordinateSystem.convert(base, system, CoordinateSystems.NAVX);
            Translation3d rotated = inNavXSystem.rotateBy(new Rotation3d(currentToZeroed));
            Translation3d inOriginalSystem = CoordinateSystem.convert(rotated, CoordinateSystems.NAVX, system);

            return inOriginalSystem;
        }
    };

    // static methods
    public static Translation3d convert(Translation3d position, CoordinateSystem system, Orientation from, Orientation to, AHRS navX2) {
        Translation3d base = from.toBase(position, system, navX2);
        return to.fromBase(base, system, navX2);
    }
}