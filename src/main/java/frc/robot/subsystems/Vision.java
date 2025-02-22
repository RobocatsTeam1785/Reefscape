package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;

    public Vision() {
        // initialization
        this.camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

        // configuration
        PhotonCamera.setVersionCheckEnabled(true);
        camera.setPipelineIndex(VisionConstants.PIPELINE);
    }
}
