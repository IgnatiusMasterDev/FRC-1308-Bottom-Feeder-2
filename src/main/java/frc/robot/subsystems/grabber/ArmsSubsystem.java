package frc.robot.subsystems.grabber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class ArmsSubsystem extends SubsystemBase {
    // Arm-extension motor
    private final TalonFX armTalon = new TalonFX(GrabberConstants.kArmCanId);

    // Encoder
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(GrabberConstants.kGrabberArmsEncoderChannelId);

    // Network tables publishing
    private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    private final NetworkTable table = networkTables.getTable("grabber");

    private final DoublePublisher encoderPublisher = table
        .getDoubleTopic("position")
        .publish();
    private final DoublePublisher anglePublisher = table
        .getDoubleTopic("angle")
        .publish();
    private final DoublePublisher velocityPublisher = table
        .getDoubleTopic("velocity")
        .publish();
    private final BooleanPublisher fullyRaisedPublisher = table
        .getBooleanTopic("fully raised")
        .publish();
    private final BooleanPublisher fullyLoweredPublisher = table
        .getBooleanTopic("fully lowered")
        .publish();

    /**
     * Creates a new ArmSubsystem with motors
     * configured to break when idle.
     */
    public ArmsSubsystem() {
        armTalon.setNeutralMode(NeutralModeValue.Brake);
        encoder.setInverted(true);
    }
    
    @Override
    public void periodic() {
        encoderPublisher.set(getPosition());
        anglePublisher.set(getAngle().getDegrees());
        velocityPublisher.set(getVelocity());
        fullyRaisedPublisher.set(fullyRaised());
        fullyLoweredPublisher.set(fullyLowered());
    }

    /**
     * Begins to raise the grabber arms. The grabber arms will not raise or will
     * stop raising if they are in the fully raised position.
     */
    public void raise() {
        if (!fullyRaised()) {
            setSpeed(GrabberConstants.kArmSpeed);
        } else {
            stop();
        }
    }

    /**
     * Begins to lower the grabber arms. The grabber arms will not lower
     * or will stop lowering if they are in the fully lowered position.
     */
    public void lower() {
        if (!fullyLowered()) {
            setSpeed(-GrabberConstants.kArmSpeed);
        } else {
            stop();
        }
    }

    /**
     * Stops the grabber arms from moving.
     */
    public void stop() {
        armTalon.set(0);
    }

    /**
     * Sets the speed of the arm talon. Should be a number between -1 and 1. Positive is
     * up and negative is down.
     * 
     * @param speed the speed to which to set the arm talon between -1 and 1.
     */
    public void setSpeed(double speed) {
        armTalon.set(speed);
    }

    /**
     * Returns whether the grabber arms are fully raised (<= 0 degrees).
     * 
     * @return true if the grabber arms are fully raised.
     */
    public boolean fullyRaised() {
        return getAngle().getDegrees() <= 0;
    }

    /**
     * Returns whether the grabber arms are fully lowered (>= 90 degrees).
     * 
     * @return true if the grabber arms are fully lowered.
     */
    public boolean fullyLowered() {
        return getAngle().getDegrees() >= 90;
    }

    /**
     * Returns the current angle of the grabber arms in degrees. This is the angle between
     * the vertical line and te grabber arms; that is, an angle of 0 is fully raised and an
     * angle of 90 is fully lowered.
     * 
     * @return The current angle of the grabber arms in degrees.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(getPosition() * -320.28 + 213.12); // TODO it would be helpful to abstract the conversion equation
    }

    /**
     * Returns the current position of the grabber arms.
     * 
     * @return The current position of the encoder in rotations.
     */
    private double getPosition() {
        return encoder.get();
    }

    /**
     * Returns the current velocity of the grabber arms. Value is betweeen -1 and 1
     * where negative is lowering the arms and positive is raising the arms.
     * 
     * @return The current velocity of the grabber arms between -1 and 1,
     */
    private double getVelocity() {
        return armTalon.get();
    }
}
