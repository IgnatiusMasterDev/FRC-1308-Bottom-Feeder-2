package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.ArmsSubsystem;

/**
 * This command moves the robot's arms to a specified angle.
 */
public class MoveArmsCommand extends Command{
    
    private final int direction;
    private final Rotation2d m_targetAngle;
    private final ArmsSubsystem m_armsSubsystem;
    private final PIDController pidController = new PIDController(.001, 0, 0);

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

        // Determine directon of movement
        // if target angle value is higher (meaning the arms are actually lower) than the actual angle
        if (m_targetAngle.getDegrees() < m_armsSubsystem.getAngle().getDegrees()) {
            direction = 1; // raise arms
        // else lower
        } else {
            direction = -1;
        }

        addRequirements(armsSubsystem);
    }

    @Override
    public void execute() {
        if (direction == 1) {
            double speed = pidController.calculate(m_armsSubsystem.getAngle().getDegrees(), m_targetAngle.getDegrees());
            m_armsSubsystem.setSpeed(speed);
        } else {
            double speed = pidController.calculate(m_armsSubsystem.getAngle().getDegrees(), m_targetAngle.getDegrees());
            m_armsSubsystem.setSpeed(-speed);
        }
    }

    @Override
    public boolean isFinished() {
        // if we are raising
        if (direction == 1) {
        // check if the arms angle is equal to or less than the target angle
            return m_armsSubsystem.getAngle().getDegrees() <= m_targetAngle.getDegrees();
        } else {
        // else check if the arms angle is equal to or greater than the target angle
            return m_armsSubsystem.getAngle().getDegrees() >= m_targetAngle.getDegrees();
        }
    }
}
