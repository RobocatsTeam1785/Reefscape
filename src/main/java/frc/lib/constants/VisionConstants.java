package frc.lib.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
    // TODO ensure this is accurate
    /** camera name, as seen in the top-right of PhotonVision's dashboard */
    public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera";

    /** the april tag pipeline index, as seen in the pipeline dropdown on the right of PhotonVision's dashboard */
    public static final int PIPELINE = 0;

    // TODO measure
    /** the height of the camera from the ground */
    public static final Distance CAMERA_HEIGHT = Inches.of(0.0);

    /** the distance of the camera from the center of the robot */
    public static final Distance CAMERA_OFFSET = Inches.of(0.0);

    /** the angle of the camera from the ground, where positive means upwards rotation */
    public static final Angle CAMERA_ANGLE = Degrees.of(0.0);

    // field april tag properties
    // TODO extract all relevant april tag heights from manual
    public static final Distance
        REEF_HEX_TAG_HEIGHT = Inches.of(0.0);
    
    // TODO extract all relevant april tag angles from manual
    // TODO determine how zero is defined
    public static final Angle
        REEF_HEX_TAG_ANGLE = Degrees.of(0.0);
}
