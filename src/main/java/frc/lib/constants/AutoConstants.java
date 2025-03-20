package frc.lib.constants;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class AutoConstants {
    // private fields
    private static final Map<Pose3d, Pose3d> leftReefSideCache = new HashMap<>();
    private static final Map<Pose3d, Pose3d> rightReefSideCache = new HashMap<>();

    // public fields
    public static final String[] ALL_AUTOS = {
        
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
        // yaw (rotation about the axis perpendicular to the floor) in radians, where counterclockwise rotation is positive, and zero is defined as away from the blue alliance driver station wall
        // see https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        double yawRadians = tag.getRotation().getZ();

        // sideways distance along the edge of the reef side, in meters
        // the april tag pose coordinates should be in the center of the april tag, and the midpoint of the two reef branches should be that center point, so the sideways distance should be
        // half the distance between the branches, which is 1 ft. 1 in., as per page 24 of the REEFSCAPE Game Manual
        // the sign argument is applied to minimize code duplication and allow reusing this method for both the left and right sides, for a sign of -1 and 1, respectively
        double sidewaysDistance = Units.inchesToMeters(sign * (12.0 + 1.0) / 2.0);

        // now, with the angle and hypotenuse of the triangle, we can calculate the opposite and adjacent sides, which correspond to the y and x offset from the april tag coordinates
        // to where the robot should be to score

        // sin(theta) = o/h
        // sin(yaw) = y offset/sideways distance
        // y offset = sideways distance * sin(yaw)
        double yOffset = sidewaysDistance * Math.sin(yawRadians);

        // cos(theta) = a/h
        // cos(yaw) = x offset/sideways distance
        // x offset = sideways distance * cos(yaw)
        double xOffset = sidewaysDistance * Math.cos(yawRadians);

        Pose3d sidePose = tag.transformBy(new Transform3d(xOffset, yOffset, 0.0, Rotation3d.kZero));
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
