// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final boolean useLimelight = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

    // SJW
    // TODO Once get a license  (then tuner x can download)
    // SignalLogger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    // SJW  Use apriltag to help update odometry
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    if(allianceOpt.isPresent() && useLimelight) {    
      Alliance alliance = allianceOpt.get();
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTableEntry x = table.getEntry("tx");
      // If dont validate tx exists warning message appear get use LimelightHelpers.getLastestResults
      if (x.exists()) {
        var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
        Pose2d llPose = (alliance == Alliance.Blue)?lastResult.getBotPose2d_wpiBlue():lastResult.getBotPose2d_wpiRed();

        if (lastResult.valid) {
          m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
        }

      }


    }

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
