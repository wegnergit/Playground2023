package frc.robot.utilities;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TestAprilTag {

    public TestAprilTag() {

    }  

    public void logAprilTagsPositions() {
        try {
            AprilTagFieldLayout layout =
                AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

            for (int i=1; i<9;i++) {
                Optional<Pose3d> pose3d = layout.getTagPose(i);
                if(pose3d.isPresent()) {
                    Pose3d pose3dFound = pose3d.get();
                    Pose2d pose = new Pose2d(
                        new Translation2d(pose3dFound.getX(),pose3dFound.getY()),
                        new Rotation2d());
                        // TODO how to get Rotation2d pose3dFound.getRotation()
                    Logger.getInstance().recordOutput("AprilTags/Tag"+Integer.toString(i)+"/Pose", pose);    
                    Logger.getInstance().recordOutput("AprilTags/Tag"+Integer.toString(i)+"/Pose3d", pose3dFound);    
                }
            }
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
