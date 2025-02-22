package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SoundSubsystem;

public class PlaySoundCommand extends Command {
    private final SoundSubsystem m_soundSubsystem;
    private final String m_filepath;

    public PlaySoundCommand(SoundSubsystem subsystem, String filepath) {
        m_filepath = filepath;
        m_soundSubsystem = subsystem;
        addRequirements(m_soundSubsystem);
    }

    @Override
    public void initialize() {
        m_soundSubsystem.loadMusic(m_filepath);
    }

    @Override
    public void execute() {
        m_soundSubsystem.play();
    }

    @Override
    public void end(boolean interrupted) {
        m_soundSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return !m_soundSubsystem.isPlaying();
    }

}
