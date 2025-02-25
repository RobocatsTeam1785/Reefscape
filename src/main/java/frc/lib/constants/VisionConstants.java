package frc.lib.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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

    /** the translation from the center of the robot to the camera */
    public static final Translation2d CAMERA_OFFSET = new Translation2d(Inches.of(0.0), Inches.of(0.0));

    /** the angle of the camera from the ground, where positive means upwards rotation */
    public static final Angle CAMERA_ANGLE = Degrees.of(0.0);

    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        CAMERA_OFFSET.getMeasureX(),
        CAMERA_OFFSET.getMeasureY(),
        CAMERA_HEIGHT,
        new Rotation3d(0.0, CAMERA_ANGLE.in(Radians), 0.0)
    );

    public static final Transform3d CAMERA_TO_ROBOT = ROBOT_TO_CAMERA.inverse();

    // field april tag properties
    // TODO write a layout for the home field
    public static final AprilTagFieldLayout COMPETITION_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // TODO extract all relevant april tag heights from manual
    public static final Distance
        REEF_HEX_TAG_HEIGHT = Inches.of(0.0);
    
    // TODO extract all relevant april tag angles from manual
    // TODO determine how zero is defined
    public static final Angle
        REEF_HEX_TAG_ANGLE = Degrees.of(0.0);
}
