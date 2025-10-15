package frc.lib.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class AutoConstants {
    // private fields
    private static final Map<Pose3d, Pose3d> leftReefSideCache = new HashMap<>();
    private static final Map<Pose3d, Pose3d> rightReefSideCache = new HashMap<>();

    // public fields
    public static final String[] ALL_AUTOS = {
        "Move"
    };

    public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Pose3d
        REEF_18 = layout.getTagPose(18).get(),

        REEF_18_LEFT = leftReefSide(REEF_18),
        REEF_18_RIGHT = rightReefSide(REEF_18);

    {
        var a = REEF_18.toMatrix();
        new Pose3d(a);
    }

    // private methods
    private static Pose3d reefSide(Pose3d tag, double sign) {
        Transform3d groundTagToRobotOnReefSide = new Transform3d(
            RobotConstants.ROBOT_LENGTH.div(2.0),
            Inches.of(-sign * (12.0 + 1.0) / 2.0),
            Meters.zero(),
            new Rotation3d(Rotation2d.k180deg)
        );

        Pose3d groundTag = new Pose3d(
            tag.getMeasureX(),
            tag.getMeasureY(),
            Meters.zero(),
            tag.getRotation()
        );

        Pose3d sidePose = groundTag.transformBy(groundTagToRobotOnReefSide);
        leftReefSideCache.put(tag, sidePose);

        return sidePose;
    }

    // public methods
    public static Pose3d leftReefSide(Pose3d tag) {
        if (leftReefSideCache.containsKey(tag)) return leftReefSideCache.get(tag);

        Pose3d sidePose = reefSide(tag, -1.0);
        leftReefSideCache.put(tag, sidePose);

        return sidePose;
    }

    public static Pose3d rightReefSide(Pose3d tag) {
        if (rightReefSideCache.containsKey(tag)) return rightReefSideCache.get(tag);

        Pose3d sidePose = reefSide(tag, 1.0);
        rightReefSideCache.put(tag, sidePose);

        return sidePose;
    }
}
