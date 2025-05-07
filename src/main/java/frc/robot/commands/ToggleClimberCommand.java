package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ToggleClimberCommand extends Command {
    private final ClimberSubsystem climber;
    private final boolean defaultUp;
    private int direction;

    /**
     * Creates a new ToggleClimberCommand.
     * 
     * @param defaultUp whether the climber should tighten (up) or loosen by default when somewhere in-between raised and lowered.
     * If true, then when the climber is not fully raised or lowered, it will raise. If false, then the climber will lower when not fully raised or
     * lowered.
     * @param climber the ClimberSubsystem to use.
     */
    public ToggleClimberCommand(boolean defaultUp, ClimberSubsystem climber) {
        this.defaultUp = defaultUp;
        this.climber = climber;
        addRequirements(climber);
    }

    /**
     * Creates a new ToggleClimberCommand.
     * 
     * @param arms the ClimberSubsystem to use.
     */
    public ToggleClimberCommand(ClimberSubsystem climber) {
        this.defaultUp = true;
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        if (climber.fullyLoosened()) {
            climber.tighten(.5);
            direction = 1;
        } else if (climber.fullyTightened()) {
            climber.loosen(2);
            direction = -1;
        } else {
            if (defaultUp) {
                climber.tighten(.5);
                direction = 1;
            } else {
                climber.loosen(1);
                direction = -1;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (direction == 1) {
            return climber.fullyTightened();
        } else {
            return climber.fullyLoosened();
        }
    }
}
