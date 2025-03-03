package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    // Both motors behave exactly the same, except one must be inverted,
    // so the motors names are arbitrary. In all cases, both are referenced.
    private final TalonFX m_talon1 = new TalonFX(ElevatorConstants.kTalon1CanId);
    private final TalonFX m_talon2 = new TalonFX(ElevatorConstants.kTalon2CanId);

    private DigitalInput topLimitSwitch = new DigitalInput(9);
    private DigitalInput bottomLimitSwitch = new DigitalInput(8);

    /**
     * Raise the elevator.
     */
    public void rise() {
        if (topLimitSwitch.get()) {
            setElevatorSpeed(ElevatorConstants.kElevatorSpeed);
        } else {
            stop();
        }
    }

    /**
     * Lower the elevator.
     */
    public void fall() {
        if (bottomLimitSwitch.get()) {
            setElevatorSpeed(-ElevatorConstants.kElevatorSpeed);
        } else {
            stop();
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
        m_talon1.set(speed);
        m_talon2.set(-speed);
    }
}
