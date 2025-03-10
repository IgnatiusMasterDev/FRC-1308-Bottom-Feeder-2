package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PhotonVision;

/**
 * This command swivels the robot until it is facing the specified AprilTag, if the
 * AprilTag is in view. This command does not move the robot translationally, although
 * the robot can be moving translationally when this command is scheduled and executed.
 */
public class AutoAimCommand extends Command {
    
    private final XboxController m_driverController;
    private final DriveSubsystem m_robotDrive;
    private final PhotonVision m_vision;
    private final int aprilTagId;

    private final double yawTolerance;
    private double forward, strafe, turn, targetYaw;
    private boolean targetVisible;

    /**
     * Constructs a new AutoAim command for the specified AprilTag.
     * 
     * @param aprilTagId the ID of the AprilTag to recognize and aim at. This command will only work for this tag.
     * @param yawTolerance the maximum error (in degrees) to tolerate being off. Otherwise, the robot will correct itself.
     * @param robot the robotContainer object.
     */
    public AutoAimCommand(int aprilTagId, double yawTolerance, RobotContainer robot) {
        m_driverController = robot.m_driverController;
        m_robotDrive = robot.m_robotDrive;
        m_vision = m_robotDrive.m_cameraVision;
        this.aprilTagId = aprilTagId;
        this.yawTolerance = yawTolerance;

        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        // Calculate drivetrain commands from Joystick values
        forward = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
        strafe = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
        turn = -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);
        targetYaw = 0.0;

        // Read relevant data from vision
        if (m_vision.aprilTagsVisible()) {
            // At least one aprilTag was seen by the camera
            for (PhotonTrackedTarget aprilTag : m_vision.getAprilTags()) {
                if (aprilTag.getFiducialId() == aprilTagId) {
                    // Found target tag, record its information
                    targetYaw = aprilTag.getYaw();
                    targetVisible = true;
                }
            }
        } else {targetVisible = false;}

        // If target apriltag is seen
        if (targetVisible && Math.abs(targetYaw) > yawTolerance) {
            // Auto-align to target
            turn = targetYaw * Configs.MAXSwerveModule.slot0Config.kP * Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond;
            m_robotDrive.drive(forward, strafe, turn, true);
        }
    }

    @Override
    public boolean isFinished() {
        // Finish if the target is not visible or we are within the angle tolerance
        return !targetVisible || Math.abs(targetYaw) < yawTolerance;
    }
}
