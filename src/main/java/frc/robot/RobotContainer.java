// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  /**
   *
   */
  private static final double DEADBAND = 0.1;
  private static final double percentSpeed = 0.2;
  private static final double percentRotatationSpeed = 0.5;
  final double MaxSpeed = 6; // 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxController joystick = new CommandXboxController(0); // My joystick
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true); // I want field-centric
                                                                                            // driving in open loop
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);
  // SJW From SwerveWithPlanner
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withIsOpenLoop(true);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * percentSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed * percentSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate * percentRotatationSpeed) // Drive counterclockwise with negative X (left)
            .withDeadband(DEADBAND)
            .withRotationalDeadband(DEADBAND)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // NOTE: No Deadband with PointWheelsAt (may need MathUtil.deadband)
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // SJW Not sure why the start robot rotated 90 degrees 
    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    
    // SJW FROM SwerveWithPlanner POV control not field centric (nice feature to drive foward or backwardof robot)
    joystick.pov(0).whileTrue(drivetrain.applyRequest(()->forwardStraight.withVelocityX(0.5*MaxSpeed).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(()->forwardStraight.withVelocityX(-0.5*MaxSpeed).withVelocityY(0)));
    joystick.pov(270).whileTrue(drivetrain.applyRequest(()->forwardStraight.withVelocityY(0.5*MaxSpeed).withVelocityX(0)));
    joystick.pov(90).whileTrue(drivetrain.applyRequest(()->forwardStraight.withVelocityY(-0.5*MaxSpeed).withVelocityX(0)));

  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
