package frc.robot.subsystems.drive;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

/**
 * Encapsulates the objects and methods associated with the robot's vision.
 */
public final class PhotonVision {
    private final PhotonCamera m_camera;
    private AprilTagFieldLayout field; // Because of a possible IO Exception, it is not possible for us to make this final, even though it should be.
    private PhotonPoseEstimator m_poseEstimator;

    /**
     * Constructs a new PhotonVision object and loads the field layout.
     */
    public PhotonVision(String cameraName) {
        m_camera = new PhotonCamera(cameraName);
        try {
            field = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/reefscape-layout-welded.json");
            m_poseEstimator = new PhotonPoseEstimator(field, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.kRobotToCamTransform);
            m_poseEstimator.setLastPose(Constants.DriveConstants.kStartingPose);
        } catch (Exception e) {
            System.out.println("Failed to load AprilTag Field Layout");
            e.printStackTrace();
        }
    }

    /**
     * Returns an estimate of the robot's pose based on the latest camera results. The
     * PhotonVision system uses a lowest ambiguity strategy to estimate pose, and take no regard
     * for what the any previous pose was.
     *
     * @return an {@link EstimatedRobotPose}, or null if no valid pose could be estimated.
     */
    public EstimatedRobotPose estimateRobotPose() {
        PhotonPipelineResult result = m_camera.getLatestResult();
        if (result.hasTargets()) {
            return m_poseEstimator.update(result).orElse(null);
        }
        return null; // No valid pose could be estimated
    }
}
