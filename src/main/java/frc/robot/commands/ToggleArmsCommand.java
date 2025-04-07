package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.ArmsSubsystem;

/**
 * Raises or lowers the robot's arms.
 */
public class ToggleArmsCommand extends Command {
    
    private final ArmsSubsystem arms;
    private final boolean defaultUp;
    private int direction;

    /**
     * Creates a new ToggleArmsCommand.
     * 
     * @param defaultUp whether the arms should raise or lower by default when they are somewhere in-between raised and lowered.
     * If true, then when the arms are not fully raised or lowered, they will raise. If false, then the arms will lower when not fully raised or
     * lowered.
     * @param arms the ArmsSubsystem to use.
     */
    public ToggleArmsCommand(boolean defaultUp, ArmsSubsystem arms) {
        this.defaultUp = defaultUp;
        this.arms = arms;
        addRequirements(arms);
    }

    /**
     * Creates a new ToggleArmsCommand.
     * 
     * @param arms the ArmsSubsystem to use.
     */
    public ToggleArmsCommand(ArmsSubsystem arms) {
        this.defaultUp = true;
        this.arms = arms;
        addRequirements(arms);
    }

    @Override
    public void initialize() {
        if (arms.fullyLowered()) {
            arms.raise();
            direction = 1;
        } else if (arms.fullyRaised()) {
            arms.lower();
            direction = -1;
        } else {
            if (defaultUp) {
                arms.raise();
                direction = 1;
            } else {
                arms.lower();
                direction = -1;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (direction == 1) {
            return arms.fullyRaised();
        } else {
            return arms.fullyLowered();
        }
    }
}
