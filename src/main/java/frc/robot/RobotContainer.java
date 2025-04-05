// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.MoveArmsCommand;
import frc.robot.commands.ToggleArmsCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.grabber.ArmsSubsystem;
import frc.robot.subsystems.grabber.WheelsSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  /* The subsystems and controllers sometimes need to be accessed by
   * commands, so they are made public so that they can be accessed. */
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public final ArmsSubsystem m_grabberArms = new ArmsSubsystem();
  public final WheelsSubsystem m_grabberWheels = new WheelsSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)),
            m_robotDrive));
    
    m_elevator.setDefaultCommand(new RunCommand(() -> m_elevator.stop(), m_elevator));
    m_grabberArms.setDefaultCommand(new RunCommand(() -> m_grabberArms.stop(), m_grabberArms));
    m_grabberWheels.setDefaultCommand(new RunCommand(() -> m_grabberWheels.stop(), m_grabberWheels));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // DRIVER BINDINGS
    // Press right bumper to set wheels in X
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    // Press down on right joystick to zero heading
    new Trigger(() -> m_driverController.getRightStickButton())
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(), m_robotDrive));

    // Press right trigger for speed mode
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.setPrecisionMode(false)));

    // Press right bumper for precision mode
    new Trigger(() -> m_driverController.getRightBumperButtonPressed())
        .onTrue(new InstantCommand(
            () -> m_robotDrive.setPrecisionMode(true)));
    
    // Hold left trigger for robot centric
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0)
        .whileTrue(new InstantCommand(
            () -> m_robotDrive.setFieldRelative(false)))
        .whileFalse(new InstantCommand(
            () -> m_robotDrive.setFieldRelative(true)));
    
    // ELEVATOR BINDINGS
    // Press right trigger to raise elevator
    new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0)
        .whileTrue(new RunCommand(
            () -> m_elevator.up(m_operatorController.getRightTriggerAxis()), m_elevator));
    // Press left trigger to lower elevator
    new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0)
        .whileTrue(new RunCommand(
            () -> m_elevator.down(m_operatorController.getLeftTriggerAxis(), false), m_elevator));

    //Press D pad up to set to floater 
    new Trigger(() -> m_operatorController.getPOV() == 0)
        .onTrue(m_elevator.getSetElevatorHeightCommand(ElevatorConstants.floaterHeight));

    //Press D Pad down to set to processor
    new Trigger(() -> m_operatorController.getPOV() == 180)
        .onTrue(m_elevator.getSetElevatorHeightCommand(ElevatorConstants.processorHeight));
        
    // Press D pad left to set to Coral 1 
    new Trigger(() -> m_operatorController.getPOV() == 270)
    .onTrue(m_elevator.getSetElevatorHeightCommand(ElevatorConstants.coral1Height));

    // Press D pad right to set to Coral 2
    new Trigger(() -> m_operatorController.getPOV() == 90)
    .onTrue(m_elevator.getSetElevatorHeightCommand(ElevatorConstants.coral2Height));
        
    // new Trigger(() -> m_driverController.getAButton())
    //     .onTrue(m_elevator.getSetElevatorHeightCommand(1.0));
    
    // GRABBER BINDINGS
    // Press right bumper to raise arms
    new Trigger(() -> m_operatorController.getRightBumperButton())
        .whileTrue(new RunCommand(
            () -> m_grabberArms.raise(), m_grabberArms));
    // Press left bumper to lower arms
    new Trigger(() -> m_operatorController.getLeftBumperButton())
        .whileTrue(new RunCommand(
            () -> m_grabberArms.lower(), m_grabberArms));
    // Press B to toggle arms up or down; 
    ToggleArmsCommand toggleArms = new ToggleArmsCommand(m_grabberArms);
    new Trigger(() -> m_operatorController.getBButton())
        .onTrue(toggleArms);
    

    // Hold A to spin grabber wheels inward
    new Trigger(() -> m_operatorController.getAButton())
    .whileTrue(new RunCommand(
        () -> m_grabberWheels.in(), m_grabberWheels));

    // Hold X to spin grabber wheels outward
    new Trigger(() -> m_operatorController.getXButton())
    .whileTrue(new RunCommand(
        () -> m_grabberWheels.out(), m_grabberWheels));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2.55, 0, new Rotation2d(0)),
        //2.7 is roughly 16 ft
        config);

        try {
            Thread.sleep(1500); //Wait 1.5 sec
        } catch (InterruptedException e){
            e.printStackTrace();
        }

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose()); // TODO figure out how to implement pose estimation instead of odometry

    // Run path following command, then stop at the end.
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, 0));
    return new SequentialCommandGroup(
        m_elevator.getSetElevatorHeightCommand(.45),
        swerveControllerCommand.andThen(() -> m_robotDrive.drive(0,0,0)),
        new MoveArmsCommand(Rotation2d.fromDegrees(45), m_grabberArms)
        );
    
  }
}