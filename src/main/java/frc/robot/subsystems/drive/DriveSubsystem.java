// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kfrontLeftCANCoderId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      false);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightCANCoderId,
      DriveConstants.kFrontRightChassisAngularOffset,
      true);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftCANCoderId,
      DriveConstants.kBackLeftChassisAngularOffset,
      false);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightCANCoderId,
      DriveConstants.kBackRightChassisAngularOffset,
      true);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kPigeonCanId);

  // Pose estimator
  private SwerveDrivePoseEstimator m_poseEstimator;

  // NetworkTable variables
  private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
  private final NetworkTable table = networkTables.getTable("drive");

  private final StructArrayPublisher<Rotation2d> headingPublisher = table
    .getStructArrayTopic("heading", Rotation2d.struct)
    .publish();
  private final StructArrayPublisher<Pose2d> posePublisher = table
    .getStructArrayTopic("pose", Pose2d.struct)
    .publish();
  private final StructArrayPublisher<SwerveModuleState> swerveStatePublisher = table
    .getStructArrayTopic("currentSwerveState", SwerveModuleState.struct)
    .publish();
  private final StructArrayPublisher<SwerveModuleState> desiredSwerveStatePublisher = table
    .getStructArrayTopic("desiredSwerveState", SwerveModuleState.struct)
    .publish();

  // Used to retrieve height percent from elevator subsystem for speed calculations
  private final NetworkTable elevatorTable = networkTables.getTable("elevator");
  private final DoubleSubscriber elevatorHeightPercentSubscription = elevatorTable.getDoubleTopic("height(percent)").subscribe(0.0);

  private boolean isPrecisionMode = false;
  private boolean isFieldRelative = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    zeroHeading();
    resetPose(DriveConstants.kStartingPose);

    // Initialize pose estimate
    m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getHeading(), getModulePositions(), AutoConstants.kStartPose); // TODO later, change to a dynamic vision call
  }

  @Override
  public void periodic() {
    
    // Publish DriveSubsystem telemetry to NetworkTables
    Pose2d[] pose = {getPose()};
    posePublisher.set(pose);

    m_poseEstimator.update(getHeading(), getModulePositions());
    
    // Publish DriveSubsystem telemetry to NetworkTables
    headingPublisher.set(new Rotation2d[] {getHeading()});
    posePublisher.set(new Pose2d[] {getPose()});
    swerveStatePublisher.set(getModuleStates());
    desiredSwerveStatePublisher.set(getDesiredModuleStates());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the robot's pose to the given pose.
   * 
   * @param pose The pose to reset to.
   */
  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPose(pose);
  }

  /**
   * Adds a pose estimation from a vision source to the DriveSubsystem's pose estimator.
   * 
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds since startup. This can be retrieved wit
   * {@code Timer.getFPGATimestamp()}.
   */
  public void addVisionMeasurement(Pose2d estimatedRobotPose, double timestampSeconds) {
    m_poseEstimator.addVisionMeasurement(estimatedRobotPose, timestampSeconds);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */  
  public void drive(double xSpeed, double ySpeed, double rot) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = calculateDelivered(
      xSpeed, 
      DriveConstants.kMaxSpeedMetersPerSecond,
      DriveConstants.kPrecisionMaxSpeedReduction,
      DriveConstants.kElevatorMaxSpeedReduction);
    double ySpeedDelivered = calculateDelivered(
      ySpeed, 
      DriveConstants.kMaxSpeedMetersPerSecond,
      DriveConstants.kPrecisionMaxSpeedReduction,
      DriveConstants.kElevatorMaxSpeedReduction);
    double rotDelivered = calculateDelivered(
      rot,  
      DriveConstants.kMaxAngularSpeed,
      DriveConstants.kPrecisionMaxRotationReduction,
      DriveConstants.kElevatorMaxRotationReduction);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getHeading())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
     
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Calculates an adjusted speed to use for the robot based off of whether the elevator is in precision mode or not
   * and the elevator height percentage.
   * 
   * @param value original speed value from controller input
   * @param maxSpeed the robot's max speed for this movement. This will differ between translational and rotational movement
   * @param precisionModeReduction the reduction factor to apply when in precision mode. This should be a value between 0 and 1 (e.g. 0.3 for 30% speed)
   * @param elevatorReduction the reduction factor to apply based on the elevator height percentage. This should also be a value between 0 and 1 (e.g. 0.5 for 50% speed reduction at max height)
   * 
   * @return the adjusted speed value.
   */
  private double calculateDelivered(double value, double maxSpeed, double precisionModeReduction, double elevatorReduction) {
    return value
        // Adjust based on max speed
         * maxSpeed
        // Reduce speed if in precision mode
         * (isPrecisionMode ? precisionModeReduction : 1)
        // Reduce speed based on elevator height if in speed mode
         * (isPrecisionMode ? 1 : 1 - (elevatorHeightPercentSubscription.get() * elevatorReduction));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading.
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Sets whether the robot should be in precision mode. In precision mode, the robot moves slower by a
   * factor of the precision reduction constant and ignores elevator height reduction.
   * 
   * @param value true if the robot should be in precision mode.
   */
  public void setPrecisionMode(boolean value) {
    isPrecisionMode = value;
  }
  
  /**
   * Sets whether the robot should move field-relative or robot-relative.
   * 
   * @param value true if the robot should move field relative, false if it should move robot-relative.
   */
  public void setFieldRelative(boolean value) {
    isFieldRelative = value;
  }

  /**
   * Returns the current chassis speeds in robot-relative coordinates. This is primarily
   * used by PathPlanner.
   * 
   * @return current robot-relative chassis speeds.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        getModuleStates());
  }

  /**
   * Sets the swerve module states.
   *
   * @param desiredStates The desired swerve module states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Returns the current swerve module states.
   * 
   * @return The current swerve module states.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  /**
   * Returns the desired swerve module states.
   * 
   * @return The desired swerve module states.
   */
  public SwerveModuleState[] getDesiredModuleStates() {
    SwerveModuleState[] desiredStates = {
      m_frontLeft.getDesiredState(),
      m_frontRight.getDesiredState(),
      m_rearLeft.getDesiredState(),
      m_rearRight.getDesiredState()
    };

    return desiredStates;
  }

  /**
   * Returns the current swerve module positions.
   * 
   * @return The current swerve module positions. 
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
  /*
  * Sets the wheels into an X formation to prevent movement.
  */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
}
