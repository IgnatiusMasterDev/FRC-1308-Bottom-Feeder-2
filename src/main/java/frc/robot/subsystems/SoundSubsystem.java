package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SoundSubsystem extends SubsystemBase {
    private static Orchestra orchestra = new Orchestra();

    /**
     * Adds an instrument to the SoundSubsystem.
     * 
     * @param instrument The device to add to the SoundSubsystem.
     * 
     * @return Status code of adding the instrument.
     */
    public static StatusCode addInstrument(ParentDevice instrument) {
        return orchestra.addInstrument(instrument);
    }

    /**
     * Loads a chirp file at the specified filepath. If the chirp file is in the deploy directory, only the name and file extension
     * is needed, e.g. "file.chrp";
     * 
     * @param filepath The path to the chirp file.
     * 
     * @return Status code of loading the chirp file.
     */
    public StatusCode loadMusic(String filepath) {
        return orchestra.loadMusic(filepath);
    }

    /**
     * Plays the loaded music file on the SoundSubsystem. If the SoundSubsystem is paused, the loaded file will resume.
     * 
     * @return Status code of playing the SoundSubsystem.
     */
    public StatusCode play() {
        return orchestra.play();
    }

    /**
     * Pauses the loaded music file on the SoundSubsystem. The position in the track will be saved so it can be resumed later.
     * 
     * @return Status code of pausing the SoundSubsystem.
     */
    public StatusCode pause() {
        return orchestra.pause();
    }

    /**
     * Stops the loaded music file on the SoundSubsystem.
     * 
     * @return Status code of stopping the SoundSubsystem
     */
    public StatusCode stop() {
        return orchestra.stop();
    }

    /**
     * Returns whether the SoundSubsystem is currently playing a file.
     * 
     * @return whether the SoundSubsystem is currently playing a file.
     */
    public boolean isPlaying() {
        return orchestra.isPlaying();
    }
}
