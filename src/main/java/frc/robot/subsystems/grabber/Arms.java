package frc.robot.subsystems.grabber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class Arms extends SubsystemBase {
    // Arm-extension motor
    private final TalonFX armTalon = new TalonFX(GrabberConstants.kArmCanId);

    private final Encoder encoder = new Encoder(2, 3, false, EncodingType.k2X);

    // Network tables publishing
    private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    private final NetworkTable table = networkTables.getTable("grabber");

    private final DoublePublisher encoderPublisher = table
        .getDoubleTopic("position")
        .publish();
    private final DoublePublisher velocityPublisher = table
        .getDoubleTopic("velocity")
        .publish();
    
    @Override
    public void periodic() {
        encoderPublisher.set(getPosition());
        velocityPublisher.set(getVelocity());
    }

    /**
     * Returns the current position of the encoder.
     * 
     * @return The current position of the encoder
     */
    private double getPosition() {
        return encoder.getDistance();
    }

    private double getVelocity() {
        return encoder.getRate();
    }

    /**
     * Begins to raise the grabber arms.
     */
    public void raiseArms() {
        armTalon.set(GrabberConstants.kArmSpeed);
    }

    /**
     * Begins to lower the grabber arms.
     */
    public void lowerArms() {
        armTalon.set(-GrabberConstants.kArmSpeed);
    }

    /**
     * Stops the grabber arms from moving.
     */
    public void stop() {
        armTalon.set(0);
    }
}
