package frc.robot.subsystems.grabber;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.GrabberConstants;

public class WheelsSubsystem extends SubsystemBase {
    // Talons that spin the wheels
    private final SparkMax leftSpark = new SparkMax(GrabberConstants.kLeftCanId, MotorType.kBrushless);
    private final SparkMax rightSpark = new SparkMax(GrabberConstants.kRightCanId, MotorType.kBrushless);

    /**
     * Begins spinning the grabber wheels inward.
     */
    public void in() {
        setWheelSpeed(GrabberConstants.kWheelSpeed / 2);
    }

    /**
     * Begins spinning the grabber wheels outward.
     */
    public void out() {
        setWheelSpeed(-GrabberConstants.kWheelSpeed);
    }

    /**
     * Stops the grabber wheels from spinning.
     */
    public void stop() {
        setWheelSpeed(0);
    }

    /**
     * Sets the wheel speeds.
     * 
     * @param speed The speed to set the wheels to. Values should be between -1.0 and 1.0.
     */
    private void setWheelSpeed(double speed) {
        // We need the wheels to spin in opposite rotations,
        // so we invert one of them.
        leftSpark.set(-speed);
        rightSpark.set(speed);
    }

    /**
     * Returns a value between -1 and 1. A positive value means the wheels
     * are spinning inward and a negative value means the
     * wheels are spinning outward.
     * 
     * @return The wheel speed between -1 and 1.
     */
    public double getWheelSpeed() {
        return rightSpark.get();
    }
}
