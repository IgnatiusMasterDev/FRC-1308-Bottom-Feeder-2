// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  // Motors
  private final TalonFX m_drivingTalon;
  private final SparkMax m_turningSpark;

  // Absolute Encoder
  private final CANcoder m_turningEncoder;

  // Motor Controllers
  private final VelocityVoltage m_talonVelocityRequest = new VelocityVoltage(0);
  private final PIDController m_turningController = new PIDController(0.3,0,0);

  private final double m_chassisAngularOffset;
  private final int m_invert;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d(0));

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, int encoderCANId, double chassisAngularOffset, boolean driveInverted) {
    // Driving Talon configuration
    m_drivingTalon = new TalonFX(drivingCANId);
    m_drivingTalon.getConfigurator().apply(Configs.MAXSwerveModule.slot0Config);
    m_drivingTalon.setPosition(0);
    m_drivingTalon.setControl(m_talonVelocityRequest);

    // Turning Spark and encoder configuration
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningController.enableContinuousInput(0, 2 * Math.PI);
    m_turningEncoder = new CANcoder(encoderCANId);

    // Set offset and inversion
    m_chassisAngularOffset = chassisAngularOffset;
    m_invert = driveInverted ? -1 : 1;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = Rotation2d.fromRadians(desiredState.angle.getRadians());

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(getAngle());

    // Command driving Talon and turning SPARK towards their respective setpoints.
    m_turningSpark.set(m_turningController.calculate(getAngle().getRadians(), correctedDesiredState.angle.getRadians()));
    // The driving Talon accepts velocity in rps, so we convert from m/s by dividing by wheel circumference.
    m_drivingTalon.setControl(m_talonVelocityRequest.withVelocity(correctedDesiredState.speedMetersPerSecond / ModuleConstants.kWheelCircumferenceMeters * m_invert));

    m_desiredState = desiredState;
  }

  /**
   * Returns the desired state of the module.
   * 
   * @return The desired state of the module.
   */
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDistance(), getAngle());
  }

  /**
   * Returns the current angle of the module.
   * 
   * @return The current angle of the module.
   */
  private Rotation2d getAngle() {
    double angle = (m_turningEncoder.getPosition().getValueAsDouble() % 1) * 2 * Math.PI - m_chassisAngularOffset;
    // we convert negative angles to positive so the angle will be between 0 and 2pi
    angle = angle > 0 ? angle : 2 * Math.PI + angle;
    return Rotation2d.fromRadians(angle);
  }

  /**
   * Returns the current velocity of the module.
   * 
   * @return The current velocity of the module in meters per second.
   */
  private double getVelocity() {
    return m_drivingTalon.getVelocity().getValueAsDouble() * ModuleConstants.kWheelCircumferenceMeters;
  }

  /**
   * Returns the distance measured by the wheel of the module.
   * 
   * @return The distance measured by the wheel of the module in meters.
   */
  private double getDistance() {
    return m_drivingTalon.getPosition().getValueAsDouble() * ModuleConstants.kWheelCircumferenceMeters;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingTalon.setPosition(0);
  }
}