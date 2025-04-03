package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.ArmsSubsystem;

/**
 * A command to move the arms to a specific place.
 */
public class MoveArmsCommand extends Command{
    
    private final Rotation2d m_targetAngle;
    private final ArmsSubsystem m_armsSubsystem;

    /**
     * Creates a new MoveArmsCommand that moves the arms to the specified angle.
     * 
     * @param angle the angle to which to move the arms. An angle of 0 is fully raised while
     * an angle equivalent to 90 degrees of pi / 2 is fully lowered. Any angle outside of that range will
     * be clamped to that range.
     */
    public MoveArmsCommand(Rotation2d targetAngle, ArmsSubsystem armsSubsystem) {
        m_armsSubsystem = armsSubsystem;

        // Clamp the target angle to the range [0, pi/2]
        if (targetAngle.getDegrees() < 0) {
            m_targetAngle = new Rotation2d(0);
        } else if (targetAngle.getDegrees() > 90) {
            m_targetAngle = new Rotation2d(90);
        } else {
            m_targetAngle = targetAngle;
        }

        addRequirements(armsSubsystem);
    }

    @Override
    public void execute() {
        // if target angle value is higher (meaning the arms are actually lower) than the actual angle
        if (m_targetAngle.getDegrees() < m_armsSubsystem.getAngle().getDegrees()) {
            m_armsSubsystem.raise();
        // else lower
        } else {
            m_armsSubsystem.lower();
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_armsSubsystem.getAngle().getDegrees() - m_targetAngle.getDegrees()) < .01;
    }
}
