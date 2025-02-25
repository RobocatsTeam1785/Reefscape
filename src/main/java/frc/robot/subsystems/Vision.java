package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.VisionConstants;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Vision extends SubsystemBase {
    // hardware
    private final PhotonCamera camera;

    // state
    private List<PhotonPipelineResult> unreadResults;

    // initialization
    public Vision() {
        // initialization
        this.camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

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

        System.out.println("number of pipeline results: " + unreadResults.size());
    }
}
