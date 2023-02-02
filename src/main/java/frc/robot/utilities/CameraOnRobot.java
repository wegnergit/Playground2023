package frc.robot.utilities;

import java.io.IOException;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.utilities.vision.estimation.CameraProperties;

public class CameraOnRobot {

    private final PhotonCamera m_PhotonCamera;
    private CameraProperties m_CameraProp;
    private final Transform3d m_RobotToCameraPose;
    private String m_CameraIpName;

    /**
     * @param cameraIpName
     * @param pipelineIndex
     * @param portToForward
     * @param configFile
     * @param cameraXPosition
     * @param cameraYPosition
     * @param cameraZPosition
     * @param cameraRotationRoll
     * @param cameraRotationPitch
     * @param cameraRotationYaw
     */
    public CameraOnRobot(String cameraIpName, 
        int pipelineIndex,
        int portToForward,
        String configFile, 
        int cameraResolutionWidth,
        int cameraResolutionHeight,
        double cameraXPosition,
        double cameraYPosition,
        double cameraZPosition,
        double cameraRotationRoll,
        double cameraRotationPitch,
        double cameraRotationYaw) {

        m_CameraIpName = cameraIpName;
        PortForwarder.add(portToForward, cameraIpName, 5800);
        m_PhotonCamera = new PhotonCamera(cameraIpName);
        m_PhotonCamera.setPipelineIndex(pipelineIndex);
        PhotonCamera.setVersionCheckEnabled(false);// TODO REMOVE so does check version

        try {
            m_CameraProp = new CameraProperties(Filesystem.getDeployDirectory()+"/"+configFile, cameraResolutionWidth, cameraResolutionHeight);
        } catch (IOException e) {
            if(m_PhotonCamera == null) {
                m_CameraProp = CameraProperties.LL2_640_480();
            }
            DriverStation.reportWarning("Unable to load configuration file for camera "+cameraIpName, e.getStackTrace());
        }
        m_RobotToCameraPose = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(cameraXPosition),
                Units.inchesToMeters(cameraYPosition),
                Units.inchesToMeters(cameraZPosition)
            ),
            new Rotation3d(
                cameraRotationRoll,
                cameraRotationPitch,
                cameraRotationYaw
            )
        );
    }

    public String getCameraIpName() {
        return m_CameraIpName;
    }

    public void setM_CameraIpName(String m_CameraIpName) {
        this.m_CameraIpName = m_CameraIpName;
    }

    public PhotonCamera getPhotonCamera() {
        return m_PhotonCamera;
    }

    public CameraProperties getCameraProp() {
        return m_CameraProp;
    }

    public Transform3d getRobotToCameraPose() {
        return m_RobotToCameraPose;
    }
    
}
