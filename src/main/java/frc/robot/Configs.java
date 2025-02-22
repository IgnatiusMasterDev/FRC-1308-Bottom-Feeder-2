package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final Slot0Configs slot0Config = new Slot0Configs();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
        public static final CANcoderConfiguration CANCoderConfig = new CANcoderConfiguration();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            // Configure PID values for Talon
            slot0Config.kV = drivingVelocityFeedForward;
            slot0Config.kP = 0.04;
            slot0Config.kI = 0;
            slot0Config.kD = 0;

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(true)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
        }
    }
}
