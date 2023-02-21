package frc.robot.utilities;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTagUtilities {
    AprilTagFieldLayout m_tagLayout;
    private Alliance m_alliance;
    public AprilTagUtilities(Alliance alliance) {

        try {
            m_tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            DriverStation.reportWarning("Unable to load ChargedUp AprilTag resource file" + 
                                        AprilTagFields.k2023ChargedUp.m_resourceFile, e.getStackTrace());
        }
        m_alliance = alliance;
    }

    public int mirrorBasedOnAlliance(int id) {
        if(mirrorIt()) {
            return mirrorId(id);
        } else {
            return id;
        }
    }

    public int mirrorId(int id) {
        switch(id) {
            case 1:
                return 8;
            case 2:
                return 7;
            case 3: 
                return 6;
            case 4: 
                return 5;
            case 5: 
                return 4;
            case 6: 
                return 3;
            case 7: 
                return 2;
            case 8:
                return 1;
        }    
        return id;
    }

    public Optional<Pose3d> getMirroredTagPose(int realId) {
        if(mirrorIt()) {
            return m_tagLayout.getTagPose(mirrorId(realId));
        }   else {
            return m_tagLayout.getTagPose(realId);
        }                   
    }

    private boolean mirrorIt() {
        return m_alliance == Alliance.Red;
    }
}
