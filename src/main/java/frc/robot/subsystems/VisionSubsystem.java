package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The VisionSubsystem is one of the more complex robot subsystems, and unlike the others,
 * exists without any commands.
 */
public class VisionSubsystem extends SubsystemBase{
    
    private final PhotonCamera camera = new PhotonCamera("camera0");
    private PhotonPipelineResult result;

    @Override
    public void periodic() {
        // Periodically updates the pipeline results from the Arducam
        result = camera.getLatestResult();
    }

    public boolean aprilTagsVisible() {
        return result.hasTargets();
    }

    public List<PhotonTrackedTarget> getAprilTags() {
        return result.getTargets();
    }
}