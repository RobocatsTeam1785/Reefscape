package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.VisionConstants;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Vision extends SubsystemBase {
    // hardware
    private final PhotonCamera camera;

    // components
    private final PhotonPoseEstimator estimator;

    // state
    private List<PhotonPipelineResult> unreadResults;

    // initialization
    public Vision() {
        // initialization
        this.camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        this.estimator = new PhotonPoseEstimator(
            VisionConstants.COMPETITION_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.ROBOT_TO_CAMERA
        );

        // configuration
        PhotonCamera.setVersionCheckEnabled(true);
        PortForwarder.add(5800, "10.17.85.11", 5800);
        
        camera.setPipelineIndex(VisionConstants.PIPELINE);

        // basic state hydration
        updateResults();
    }

    // state modification
    /** updates the unread pipeline results */
    private void updateResults() {
        unreadResults = camera.getAllUnreadResults();
    }

    // state
    /** returns the latest unread pipeline result, or null if unreadResults is empty */
    @Logged
    public PhotonPipelineResult latestResult() {
        int index = unreadResults.size() - 1;
        return index >= 0 ? unreadResults.get(index) : null;
    }

    @Override
    public void periodic() {
        updateResults();

        if (unreadResults.size() > 0) {
            PhotonPipelineResult result = latestResult();
            
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                Optional<Pose3d> targetPose = VisionConstants.COMPETITION_LAYOUT.getTagPose(target.getFiducialId());

                if (targetPose.isPresent()) {
                    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                        target.getBestCameraToTarget(),
                        targetPose.get(),
                        VisionConstants.CAMERA_TO_ROBOT
                    );
                }
            }
        }
    }
}
