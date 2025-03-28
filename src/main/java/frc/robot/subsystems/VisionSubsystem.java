package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;


public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera m_Camera = new PhotonCamera("Arducam_OV2311_USB_Camera");
    private ArrayList<PhotonPipelineResult> m_LatestResults = new ArrayList<>();
    private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    private final NetworkTable table = networkTables.getTable("vision");
    private int m_LastDetectedId = -1;

    private final IntegerPublisher lastDetectedIdPublisher = table
        .getIntegerTopic("last detected id")
        .publish();

    public VisionSubsystem() {

    }

    @Override
    public void periodic() {
        var results = m_Camera.getAllUnreadResults();
        if (results.size() > 0) {
            m_LatestResults.clear();
            m_LatestResults.addAll(results);
        }
        lastDetectedIdPublisher.set(getLastDetectedTargetID());
    }

    public int getLastDetectedTargetID() {
        m_LatestResults.forEach((result) -> {
            if(result.hasTargets()) {
                m_LastDetectedId = result.getBestTarget().getFiducialId();
            }
        });
        return m_LastDetectedId;
    }
}