package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class VisionSubsystem extends SubsystemBase {

    private final RobotContainer m_container;

    private final UsbCamera arducam = Robot.arducam;
    private final CvSink cvSink = CameraServer.getVideo();

    private final AprilTagDetector detector = new AprilTagDetector();
    private final AprilTagPoseEstimator.Config config = new AprilTagPoseEstimator.Config(.206, 361.6942, 359.185383, 320, 240);
    private final AprilTagPoseEstimator poseEstimator = new AprilTagPoseEstimator(config);

    public VisionSubsystem(RobotContainer container) {
        m_container = container;
        // initialize arducam
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
                System.out.println("Detected tag ID: " + detection.getId());
                Transform3d transform = poseEstimator.estimate(detection);
                Pose2d pose = new Pose2d(transform.getX(), transform.getY(), new Rotation2d(transform.getRotation().getZ()));
                m_container.m_robotDrive.addVisionMeasurement(pose, Timer.getFPGATimestamp());
            }
        }
    }
}