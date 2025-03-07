package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.WheelsSubsystem;

/**
 * Toggles the grabber wheels on or off. If the grabber wheels are
 * currently spinning and a ToggleWheelsCommand is issued for the opposite direction,
 * then the wheels will begin to spin in the opposite direction rather than
 * stop.
 */
public class ToggleWheelsCommand extends Command {

    private int direction;
    private WheelsSubsystem wheelsSubsystem;
    
    /**
     * Creates a new ToggleWheelsCommand.
     * 
     * @param inward whether the wheels should spin inward or outward.
     * @param wheelsSubsystem the wheelsSubsystem to use.
     */
    public ToggleWheelsCommand(boolean inward, WheelsSubsystem wheelsSubsystem) {
        addRequirements(wheelsSubsystem);
        this.wheelsSubsystem = wheelsSubsystem;
        direction = inward ? 1 : -1;
    }

    @Override
    public void initialize() {
        if (wheelsSubsystem.getWheelSpeed() * direction > 0) {
            wheelsSubsystem.stop();
        } else {
            if (direction == 1) {
                wheelsSubsystem.in();
            } else {
                wheelsSubsystem.out();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
