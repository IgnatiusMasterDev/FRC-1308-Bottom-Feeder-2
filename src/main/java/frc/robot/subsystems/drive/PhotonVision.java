package frc.robot.subsystems.drive;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;

/**
 * Encapsulates the objects and methods associated with the robot's vision.
 * This class is only loosely associated with the drive system, but that's the best
 * I could come up with for where to put this.
 */
public class PhotonVision {
    private final PhotonCamera m_camera = new PhotonCamera(VisionConstants.kPhotonCameraName);
    private AprilTagFieldLayout field; // Because of a possible IO Exception, it is not possible for us to make this final, even though it should be.
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(field, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, VisionConstants.kRobotToCamTransform);

    private Pose3d prevEstimatedPose;

    /**
     * Constructs a new PhotonVision object and loads the field layout.
     */
    public PhotonVision() {
        try {
            field = new AprilTagFieldLayout("reefscape-layout-welded.json");
        } catch (Exception e) {
            System.out.println("Failed to load AprilTag Field Layout");
            e.printStackTrace();
        }
    }

    /**
     * Returns the robot's pose as estimated from PhotonVision, wrapped in an Optional EstimatedRobotPose.
     * To access the actual Pose3d, use {@code getEstimatedPose().get().estimatedPose}. If no AprilTags
     * are in sight, then the Option.get() method throws an error.
     * 
     * @return the robot's pose wrapped in an Optional EstimatedRobotPose.
     */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        if (prevEstimatedPose != null) {
            poseEstimator.setReferencePose(prevEstimatedPose);
        }

        Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(getLatestResult());
        try {
            prevEstimatedPose = estimatedRobotPose.get().estimatedPose;
        } catch (Exception e) {}
        return estimatedRobotPose;
    }

    /**
     * Returns a list of visible AprilTags as {@link PhotonTrackedTarget}s. If no AprilTags are currently visible,
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
        return m_camera.getLatestResult(); // TODO PhotonCamera.getLatestResult() is deprecated
    }
}
