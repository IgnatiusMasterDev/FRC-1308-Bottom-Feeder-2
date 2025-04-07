package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.ArmsSubsystem;

/**
 * Command to move the robot's arms to a specified angle.
 * 
 * <p>An angle of 0 degrees corresponds to fully raised arms, while 90 degrees corresponds to fully lowered arms.
 * Any angle outside this range will be clamped.
 */
public class MoveArmsCommand extends Command {
    private final ArmsSubsystem m_armsSubsystem;
    private final Rotation2d m_targetAngle;
    private final PIDController m_pidController;

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

        addRequirements(armsSubsystem);
    }

    @Override
    public void initialize() {
        // Reset the PID controller to ensure a smooth start
        m_pidController.reset();
    }

    @Override
    public void execute() {
        // Get the current angle of the arms
        double currentAngle = m_armsSubsystem.getAngle().getDegrees();

        // Calculate the output speed using the PID controller
        double speed = m_pidController.calculate(currentAngle);

        // Apply a feedforward term to improve responsiveness
        double feedforward = 0.05 * Math.signum(m_targetAngle.getDegrees() - currentAngle);

        // Set the arm speed, combining PID output and feedforward
        m_armsSubsystem.setSpeed(speed + feedforward);
    }

    @Override
    public boolean isFinished() {
        // Finish when the arms are within the tolerance of the target angle
        return m_pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the arms when the command ends
        m_armsSubsystem.stop();
    }

    /**
     * Clamps the target angle to the range [0, 90] degrees.
     * 
     * @param angle The desired target angle.
     * 
     * @return A clamped Rotation2d object.
     */
    private Rotation2d clampAngle(Rotation2d angle) {
        double clampedDegrees = Math.max(0, Math.min(90, angle.getDegrees()));
        return Rotation2d.fromDegrees(clampedDegrees);
    }
}