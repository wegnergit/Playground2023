package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import java.util.Optional;
import java.util.function.Consumer;

/**
 * Gives the robot an odometry position when it starts in teleop if it doesn' already have a
 * position
 */
public class StartInTeleopUtility {
  // TODO switch field
  private AprilTagFieldLayout m_AprilTagFieldLayout =
      //   AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private Consumer<Pose2d> m_ResetPose;

  private Pose2d temp;
  private Pose2d pose;

  private boolean m_HasRunAutonomous = false;
  private boolean m_HasSeenTags = false;

  private Trigger m_checkIfAllianceChangedTrigger = null;

  private boolean m_isFirstTime = true;

  /**
   * Resets the robots position.
   *
   * @param resetPose
   */
  public StartInTeleopUtility(Consumer<Pose2d> resetPose) {
    m_ResetPose = resetPose;
  }

  /** Lets us know that we have run in autonomous */
  public void updateAutonomous() {
    m_HasRunAutonomous = true;
  }

  /** lets us know that we have seen april tags */
  public void updateTags() {
    m_HasSeenTags = true;
  }

  /** Sets the robot position based on the alliance if there is one. */
  public void setRobotPositionBasedOnAlliance() {
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
    if (optionalAlliance.isPresent()) {
      Alliance alliance = optionalAlliance.get();
      if (alliance == Alliance.Red) {
        // temp = m_AprilTagFieldLayout.getTagPose(7).get().toPose2d();
        // https://github.com/wpilibsuite/allwpilib/blob/b65f159c3fdfdfce9ec126cf79dfd9a16d1e66e9/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape.json
        // ID: 7
        temp =
            new Pose3d(
                    13.890498, 4.0259, 0.308102, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))
                .toPose2d();
        pose =
            new Pose2d(
                temp.getX() + 2.0,
                temp.getY(),
                temp.getRotation().plus(Rotation2d.fromDegrees(180.0)));
      } else {
        // temp = m_AprilTagFieldLayout.getTagPose(18).get().toPose2d();
        // https://github.com/wpilibsuite/allwpilib/blob/b65f159c3fdfdfce9ec126cf79dfd9a16d1e66e9/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape.json
        // ID: 18
        temp =
            new Pose3d(
                    3.6576,
                    4.0259,
                    0.308102,
                    new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))
                .toPose2d();
        pose =
            new Pose2d(
                temp.getX() - 2.0,
                temp.getY(),
                temp.getRotation().plus(Rotation2d.fromDegrees(180.0)));
      }

      m_ResetPose.accept(pose);
    }
  }

  /** Create a trigger and waits to update the alliance position when it is available */
  public void createTriggerForSimulation() {
    if (m_checkIfAllianceChangedTrigger == null) {
      m_checkIfAllianceChangedTrigger =
          new Trigger(() -> checkIsAlliancePresent())
              .onTrue(
                  (new InstantCommand(() -> setRobotPositionBasedOnAlliance()))
                      .ignoringDisable(true));
    }
  }

  /**
   * Returns the alliance
   *
   * @return
   */
  private boolean checkIsAlliancePresent() {
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
    if (m_isFirstTime) {
      m_isFirstTime = false;
      return false;
    }
    return optionalAlliance.isPresent();
  }

  /**
   * If auto hasn't been run and the robot hasn't seen april tags, it will then update the odoemtry
   * when it enters teleop
   */
  public void updateStartingPosition() {
    if (m_HasRunAutonomous == false && m_HasSeenTags == false) {
      if (Robot.isReal()) {
        setRobotPositionBasedOnAlliance();
      } else {
        createTriggerForSimulation();
      }
    }
  }
}
