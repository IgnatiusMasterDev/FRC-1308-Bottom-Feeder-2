package frc.robot.subsystems.drive;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Encapsulates the objects and methods associated with the robot's vision.
 */
public class PhotonVision {
    private final PhotonCamera m_camera = new PhotonCamera("camera0");
    
    /**
     * Returns a list of visible AprilTags. If no AprilTags are currently visible,
     * then this method returns {@code null}, so usually {@code aprilTagsVisible()} must be
     * called before this method is called.
     * 
     * @return a list of visible and tracked AprilTags.
     */
    public List<PhotonTrackedTarget> getAprilTags() {
        return getLatestResult().getTargets();
    }

    /**
     * Returns true if at least one AprilTag is visible and being tracked.
     * 
     * @return true if at least one AprilTag is visible and being tracked.
     */
    public boolean aprilTagsVisible() {
        return getLatestResult().hasTargets();
    }

    /**
     * Returns the latest {@link PhotonPipelineResult} from the robot's camera and coprocessor.
     * 
     * @return the latest {@link PhotonPipielineResult}.
     */
    private PhotonPipelineResult getLatestResult() {
        return m_camera.getLatestResult();
    }
}
