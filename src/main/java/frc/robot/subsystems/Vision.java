package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.VisionConstants;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Vision extends SubsystemBase {
    // hardware
    public final PhotonCamera camera;

    // components
    public final PhotonPoseEstimator estimator;

    // state
    public PhotonPipelineResult latestResult;

    // initialization
    public Vision() {
        // initialization
        this.camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        this.estimator = new PhotonPoseEstimator(
            VisionConstants.COMPETITION_LAYOUT,
            // TODO configure AprilTagFieldLayout properly in the PhotonVision UI
            // TODO using this: https://docs.photonvision.org/en/v2025.2.1-rc2/docs/apriltag-pipelines/multitag.html#multitag-localization
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.ROBOT_TO_CAMERA
        );

        // configuration
        PhotonCamera.setVersionCheckEnabled(true);
        PortForwarder.add(5800, "10.17.85.11", 5800);
        
        camera.setPipelineIndex(VisionConstants.PIPELINE);
    }

    // state
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousEstimatedRobotPose) {
        if (latestResult == null) return Optional.empty();

        estimator.setReferencePose(previousEstimatedRobotPose);
        return estimator.update(latestResult);
    }

    // periodic
    @Override
    public void periodic() {
        // update latest result
        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
        
        if (unreadResults.size() > 0) {
            latestResult = unreadResults.get(unreadResults.size() - 1);
        }
    }
}
