// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Configs;
import frc.robot.Constants;

public class MAXSwerveModule {
  private final TalonFX m_drivingTalon;
  private final SparkMax m_turningSpark;
  private final CANcoder m_turningEncoder;

  // Talon specific vars
  private final VelocityVoltage m_talonVelocityRequest = new VelocityVoltage(0);
  private SparkClosedLoopController m_turningClosedLoopController;


  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d(0));

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, int encoderCANId, double chassisAngularOffset) {
    m_drivingTalon = new TalonFX(drivingCANId);
    m_drivingTalon.getConfigurator().apply(Configs.MAXSwerveModule.slot0Config);
    m_drivingTalon.setPosition(0);
    m_drivingTalon.setControl(m_talonVelocityRequest); // set driving velocity to 0. We only need to do this for Talon.

    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    m_turningEncoder = new CANcoder(encoderCANId);

    m_chassisAngularOffset = chassisAngularOffset;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingTalon.getVelocity().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition().getValueAsDouble() - m_chassisAngularOffset));
  }

  /**
   * Returns the desired state of the module.
   * 
   * @return the desired state of the module.
   */
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingTalon.getPosition().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition().getValueAsDouble() - m_chassisAngularOffset));
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
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(getPosition().angle);

    // Command driving Talon and turning SPARK towards their respective setpoints.
    // The driving Talon accepts velocity in rps, so we convert from m/s by dividing by wheel circumference.
    m_drivingTalon.setControl(m_talonVelocityRequest.withVelocity(correctedDesiredState.speedMetersPerSecond / Constants.ModuleConstants.kWheelCircumferenceMeters));
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians() / (2 * Math.PI), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingTalon.setPosition(0);
  }
}
