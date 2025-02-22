package frc.lib.logging;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(PhotonPipelineResult.class)
public class PhotonPipelineResultLogger extends ClassSpecificLogger<PhotonPipelineResult> {
    public PhotonPipelineResultLogger() {
        super(PhotonPipelineResult.class);
    }

    @Override
    public void update(EpilogueBackend backend, PhotonPipelineResult result) {
        // TODO make this never possible, if possible
        if (result == null) return;

        boolean hasTargets = result.hasTargets();
        int targetCount = hasTargets ? result.getTargets().size() : 0;
        PhotonTrackedTarget bestTarget = hasTargets ? result.getBestTarget() : null;

        // target presence and count
        backend.log("Has targets", hasTargets);
        backend.log("Target count", targetCount);

        // best target
        backend.log("Best target yaw", bestTarget.yaw);
        backend.log("Best target pitch", bestTarget.pitch);
        backend.log("Best target skew", bestTarget.skew);

        backend.log("Best target area", bestTarget.area);
        backend.log("Best target pose ambiguity", bestTarget.poseAmbiguity);
        
        backend.log("Best target fiducial id", bestTarget.fiducialId);
        backend.log("Best target object detection id", bestTarget.objDetectId);
    }
}
