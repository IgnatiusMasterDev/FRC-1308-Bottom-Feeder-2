package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ClimberSubsystem extends SubsystemBase{
    private final TalonFX motor = new TalonFX(21);

    private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    private final NetworkTable table = networkTables.getTable("climber");

    private final double tightenLimit = 137.453;
    private final double loosenLimit = 8.741;
    private final double percentileFactor = (tightenLimit - loosenLimit);

    private final DoublePublisher positionPublisher = table
        .getDoubleTopic("position (rot)")
        .publish();

    private final DoublePublisher positionPercentilePublisher = table
        .getDoubleTopic("position (percentile)")
        .publish();

    public ClimberSubsystem() {
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        // Update the position publisher with the current position of the motor
        positionPublisher.set(motor.getPosition().getValueAsDouble());
        positionPercentilePublisher.set(getPosition());
    }

    /**
     * Loosens the climber.
     * 
     * @param speed between 0 and 1.
     */
    public void loosen(double speed) {
        if (!fullyLoosened()) {
            setSpeed(-speed);
        } else {
            stop();
        }
    }

    /**
     * Tightens the climber.
     * 
     * @param speed between 0 and 1.
     */
    public void tighten(double speed) {
        if (!fullyTightened()) {
            setSpeed(speed);
        } else {
            stop();
        }
    }

    /** Stops the climber. */
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Set the speed of the climber's motors.
     * 
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    /**
     * Returns whether the climber is fully tightened (pulled back).
     * 
     * @return whether the climber is fully released.
     */
    public boolean fullyTightened() {
        return getPosition() >= 1;
    }

    /**
     * Returns whether the climber is fully loosened (released).
     * 
     * @return whether the climber is fully released.
     */
    public boolean fullyLoosened() {
        return getPosition() <= 0;
    }

    /**
     * Returns the position of the climber as a percentile of the loose-tight range.
     * 
     * @return the position of the climber as a percentile.
     */
    public double getPosition() {
        return (motor.getPosition().getValueAsDouble() - loosenLimit) / percentileFactor; 
    }
}
