package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    // Both motors behave exactly the same, except one must be inverted,
    // so the motors names are arbitrary. In all cases, both are referenced.
    private final TalonFX m_talon1 = new TalonFX(ElevatorConstants.kTalon1CanId);
    private final TalonFX m_talon2 = new TalonFX(ElevatorConstants.kTalon2CanId);

    // private DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.kTopLimitSwitchChannelId);
    private DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.kBottomLimitSwitchChannelId);

    // Encoder and positioning variables
    // private final DutyCycleEncoder encoder = new DutyCycleEncoder(ElevatorConstants.kElevatorEncoderChannelId);
    private final Encoder encoder = new Encoder(ElevatorConstants.kElevatorEncoderChannelAId,
                                                ElevatorConstants.kElevatorEncoderChannelBId,
                                                false,
                                                Encoder.EncodingType.k1X);
    private boolean isCalibrated = false;
    private double lastSetSpeed = 0.0;

    // Network tables publishing
    private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    private final NetworkTable table = networkTables.getTable("elevator");

    private final DoublePublisher encoderPublisher = table
        .getDoubleTopic("position")
        .publish();
    private final DoublePublisher velocityPublisher = table
        .getDoubleTopic("velocity")
        .publish();
    private final DoublePublisher lastSetSpeedPublisher = table
        .getDoubleTopic("lastSetSpeed")
        .publish();
    // private final BooleanPublisher topLimitSwitchPublisher = table
    //     .getBooleanTopic("top limit switch")
    //     .publish();
    private final BooleanPublisher bottomLimitSwitchPublisher = table
        .getBooleanTopic("bottom limit switch")
        .publish();

    /**
     * Creates a new ElevatorSubsystem with the elevator motors
     * configured to break on idle.
     */
    public ElevatorSubsystem() {
        m_talon1.setNeutralMode(NeutralModeValue.Brake);
        m_talon2.setNeutralMode(NeutralModeValue.Brake);
        encoder.setDistancePerPulse(ElevatorConstants.kDistancePerPulse);
    }

    @Override
    public void periodic() {
        // Publish values to NetworkTables
        encoderPublisher.set(getPosition());
        velocityPublisher.set(getVelocity());
        // topLimitSwitchPublisher.set(atTop());
        bottomLimitSwitchPublisher.set(atBottom());
        lastSetSpeedPublisher.set(lastSetSpeed);

        // Calibrate elevator if it needs to be
        if (!isCalibrated) {
            new InstantCommand(() -> down(ElevatorConstants.kElevatorHomingSpeed, true), this).schedule();
            if (atBottom()) {
                isCalibrated = true;
                encoder.reset();
                stop();
            }
        }
    }

    /**
     * Returns the position of the elevator.
     * 
     * <p>This method MUST be called in the periodic method in order to function properly.
     * 
     * @return the position of the elevator in rotations.
     */
    private double getPosition() {
        return encoder.getDistance() + ElevatorConstants.kHeightOffset;
    }

    /**
     * Returns the current speed of the elevator as a value between -1 and 1 where
     * a negative value is going down and a positive value is up.
     * 
     * @return the velocity of the elevator. Values are between -1 and 1.
     */
    private double getVelocity() {
        return m_talon1.get();
    }


    /**
     * Begin raising the elevator. The elevator will not rise or will stop moving if the
     * top limit switch is pressed.
     *  
     * @return true if the elevator is rising.
     */
    
    public boolean up(double speed) {
        double ratio = (getPosition() - ElevatorConstants.kHeightOffset) / (ElevatorConstants.kMaxHeight - ElevatorConstants.kHeightOffset);
        boolean inAttenuationZone = ratio >= (1.0 - ElevatorConstants.kAttenuationBand);
        double speedMultiplier = inAttenuationZone ? ElevatorConstants.kAttenuationMultiplier : 1.0;
        double newSpeed = (ElevatorConstants.kAlpha * speedMultiplier * speed) + 
            ((1.0 - ElevatorConstants.kAlpha) * lastSetSpeed);
        
        if (!atTop()) {
            setElevatorSpeed(newSpeed);
            return true;
        } else {
            stop();
            return false;
        }
    }

    /**
     * Begin lowering the elevator. The elevator will not lower or
     * will stop moving if the bottom limit switch is pressed.
     * 
     * @return true if the elevator is lowering.
     */
    public boolean down(double speed, boolean isHoming) {
        double ratio = (getPosition() - ElevatorConstants.kHeightOffset) / (ElevatorConstants.kMaxHeight - ElevatorConstants.kHeightOffset);
        boolean inAttenuationZone = ratio <= ElevatorConstants.kAttenuationBand;
        double speedMultiplier = (!isHoming && inAttenuationZone) ? ElevatorConstants.kAttenuationMultiplier : 1.0;
        double newSpeed = (ElevatorConstants.kAlpha * speedMultiplier * -speed) + 
            ((1.0 - ElevatorConstants.kAlpha) * lastSetSpeed);

        if (!atBottom()) {
            setElevatorSpeed(newSpeed);
            return true;
        } else {
            stop();
            return false;
        }
    }

    /**
     * Stop the elevator from moving.
     */
    public void stop() {
        setElevatorSpeed(0);
    }

    /**
     * Set the speed of the elevator's motors.
     * 
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    private void setElevatorSpeed(double speed) {
        // The motors have opposite orientations on the robot,
        // so one must be inverted to spin in the same direction
        lastSetSpeed = speed;
        m_talon1.set(speed);
        m_talon2.set(-speed);
    }

    /**
     * Returns true if the elevator is as far up as it can be.
     * 
     * @return true if the elevator is all the way up
     */
    private boolean atTop() {
        // return !topLimitSwitch.get() || getPosition() >= ElevatorConstants.kMaxHeight;
        return getPosition() >= ElevatorConstants.kMaxHeight;
    }

    /**
     * Returns true if the elevator is as far down as it can be.
     * 
     * @return true if the elevator is all the way down.
     */
    private boolean atBottom() {
        return bottomLimitSwitch.get();
    }
}
