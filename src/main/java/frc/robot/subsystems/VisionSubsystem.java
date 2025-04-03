package frc.robot.subsystems;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


public class VisionSubsystem extends SubsystemBase {

    private final UsbCamera arducam = Robot.arducam;
    private final CvSink cvSink = CameraServer.getVideo();

    private final AprilTagDetector detector = new AprilTagDetector();
    private final AprilTagPoseEstimator.Config config = new AprilTagPoseEstimator.Config(.206, 361.6942, 359.185383, 320, 240);
    private final AprilTagPoseEstimator poseEstimator = new AprilTagPoseEstimator(config);

    private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    private final NetworkTable table = networkTables.getTable("vision");

    public VisionSubsystem() {
        // initialize webcam
        arducam.setResolution(640, 480);
        arducam.setConnectionStrategy(UsbCamera.ConnectionStrategy.kKeepOpen); 

        detector.addFamily("tag36h11");
    }

    @Override
    public void periodic() {
        Mat frame = new Mat();
        if (cvSink.grabFrame(frame) > 0) {
            AprilTagDetection[] detections = detector.detect(frame);
            for (AprilTagDetection detection : detections) {
                System.out.println("Detection tag ID: " + detection.getId());

                Transform3d estimatedPose = poseEstimator.estimate(detection);
            }
        }
    }
}