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
     * Creates a new MoveArmsCommand to move the arms to the specified angle.
     * 
     * @param targetAngle The target angle for the arms. Clamped to the range [0, 90] degrees. An
     * angle of 0 degrees is fully raised and 90 degrees is fully lowered.
     * @param toleranceDegrees The tolerance range in degrees for the command's termination.
     * @param armsSubsystem The ArmsSubsystem controlling the robot's arms.
     */
    public MoveArmsCommand(Rotation2d targetAngle, ArmsSubsystem armsSubsystem) {
        this(targetAngle, 1.0, armsSubsystem);
    }
    
    /**
     * Creates a new MoveArmsCommand to move the arms to the specified angle.
     * 
     * @param targetAngle The target angle for the arms. Clamped to the range [0, 90] degrees. An
     * angle of 0 degrees is fully raised and 90 degrees is fully lowered.
     * @param toleranceDegrees The tolerance range in degrees for the command's termination.
     * @param armsSubsystem The ArmsSubsystem controlling the robot's arms.
     */
    public MoveArmsCommand(Rotation2d targetAngle, double toleranceDegrees, ArmsSubsystem armsSubsystem) {
        m_armsSubsystem = armsSubsystem;
        m_targetAngle = clampAngle(targetAngle);
        m_pidController = new PIDController(0.02, 0.0, 0.001); // Tuned PID values for smoother movement
        m_pidController.setTolerance(toleranceDegrees); // Allowable error in degrees
        m_pidController.setSetpoint(m_targetAngle.getDegrees());

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
    public void initialize() {
        // Reset the PID controller to ensure a smooth start
        m_pidController.reset();
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