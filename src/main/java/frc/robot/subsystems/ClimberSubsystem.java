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

    private final DoublePublisher positionPublisher = table
        .getDoubleTopic("position")
        .publish();

    public ClimberSubsystem() {
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        // Update the position publisher with the current position of the motor
        positionPublisher.set(getPosition());
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
     * Returns the position of the climber.
     * 
     * @return the position of the climber
     */
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }
}
