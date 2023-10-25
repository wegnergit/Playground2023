// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.AutoCommandManager;
import frc.robot.autos.AutoCommandManager.subNames;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.SlapstickCommand;
import frc.robot.commands.SwerveLockCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.MechanismSimulator;
import frc.robot.subsystems.SlapstickSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TopRollerSubsystem;
import frc.robot.subsystems.arm.ArmIORobot;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorIORobot;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.utilities.CommandFactoryUtility;
import frc.robot.utilities.RobotInformation;
// import frc.robot.utilities.TimeOfFlightUtility;
import frc.robot.utilities.RobotInformation.WhichRobot;
import frc.robot.utilities.TargetScorePositionUtility.Target;
import frc.robot.utilities.SwerveModuleConstants;
import frc.robot.utilities.TargetScorePositionUtility;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  Alliance alliance = null; // NOLONGER exists Alliance.Invalid;
  boolean allianceAssigned = false;

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;   
  private final int rotationAxis = XboxController.Axis.kRightX.value;


    // Which Robot code should we use? competition or not
    //Cannot use an ID of 0
    //Changed the turningMotorID and cancoderID from 0 to 3
    //https://buildmedia.readthedocs.org/media/pdf/phoenix-documentation/latest/phoenix-documentation.pdf
    //page 100
    RobotInformation robotInfo = 
        // Non-Competition robot attributes
        new RobotInformation(WhichRobot.PRACTICE_ROBOT,
          new SwerveModuleConstants(8, 9, 9, 113.818), 
          new SwerveModuleConstants(11, 10, 10, 232.031), 
          new SwerveModuleConstants(1, 3, 3, 89.033), 
          new SwerveModuleConstants(18, 19, 19, 6.064)); 

  public static final int kDriverControllerPort = 0;
  public static final int kCodriverControllerPort = 1;
  
  // Creates air pressure for pistons to work
  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
   
 /* Modules */
  public final SwerveModuleConstants frontLeftModule = robotInfo.getFrontLeft();
  public final SwerveModuleConstants frontRightModule =  robotInfo.getFrontRight();
  public final SwerveModuleConstants backLeftModule = robotInfo.getBackLeft();
  public final SwerveModuleConstants backRightModule = robotInfo.getBackRight();
  
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);
  CommandXboxController m_codriverController = new CommandXboxController(kCodriverControllerPort);

  // Subsystems \\
  private final SwerveDrive m_robotDrive = new SwerveDrive(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final FieldSim m_fieldSim = new FieldSim(m_robotDrive);
  private final TeleopSwerve m_TeleopSwerve = new TeleopSwerve(m_robotDrive, m_driverController, translationAxis, strafeAxis, rotationAxis, true, true);

  //private final PitchIntakeSubsystem m_PitchIntakeSubsystem = new PitchIntakeSubsystem(Robot.isReal()? new PitchIntakeIORobot(14): new PitchIntakeIOSim());
  
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(Robot.isReal() ? new ArmIORobot(2, 4) : new ArmIOSim());
  private final ManipulatorSubsystem m_manipulatorSubsystem = new ManipulatorSubsystem(new ManipulatorIORobot(6, 7));
  private final TopRollerSubsystem m_topRollerSubsystem = new TopRollerSubsystem(5, 0);
  private final MechanismSimulator m_mechanismSimulator = new MechanismSimulator(m_armSubsystem, /*m_PitchIntakeSubsystem,*/ m_robotDrive);
  private final SlapstickSubsystem m_slapstickSubsystem = new SlapstickSubsystem(1);

  // Utilities \\
  // private final TimeOfFlightUtility m_timeOfFlight = new TimeOfFlightUtility(1);
  private TargetScorePositionUtility m_targetScorePositionUtility = new TargetScorePositionUtility();
  
  // Commands \\
  //private final RotateCommand m_rotateCommand = new RotateCommand(new Pose2d( 8.2423, 4.0513, new Rotation2d(0.0)), m_robotDrive);
  private final AutoBalanceCommand m_autoBalanceCommand = new AutoBalanceCommand(m_robotDrive);
  private final SwerveLockCommand m_SwerveLockCommand = new SwerveLockCommand(m_robotDrive, true);
  private Command m_groundIntakeCommand = CommandFactoryUtility.createIntakeCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem);
  private Command m_groundIntakeAndSlowDownCommand = CommandFactoryUtility.createIntakeAndSlowDownCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem, m_TeleopSwerve);
  private Command m_stowArmCommand = CommandFactoryUtility.createStowArmCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem);
  private Command m_stowArmAndSpeedUpCommand = CommandFactoryUtility.createStowArmAndSpeedUpCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem, m_TeleopSwerve);
  private Command m_createScoreLowCommand = CommandFactoryUtility.createScoreLowCommand(m_armSubsystem, m_manipulatorSubsystem);
  private Command m_createScoreMediumCommand = CommandFactoryUtility.createScoreMediumCommand(m_armSubsystem, m_manipulatorSubsystem);
  private Command m_createScoreHighCommand = CommandFactoryUtility.createScoreHighCommand(m_armSubsystem, m_manipulatorSubsystem);

    
  private AutoCommandManager m_autoManager;
  private Map<String, Command> eventCommandMap = new HashMap<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    checkDSUpdate();

    // Auto Commands
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "intakeCube", 
        m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "stowArm", 
        m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "shootCube", 
        m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem);

    // eventCommandMap = new HashMap<>();

    m_autoManager = new AutoCommandManager();
    m_autoManager.addSubsystem(subNames.SwerveDriveSubsystem, m_robotDrive);
    m_autoManager.addSubsystem(subNames.ArmSubsystem, m_armSubsystem);
    m_autoManager.addSubsystem(subNames.ManipulatorSubsystem, m_manipulatorSubsystem);
    m_autoManager.addSubsystem(subNames.TopRollerSubsystem, m_topRollerSubsystem);
    m_autoManager.initCommands(eventCommandMap);

    // Configure the button bindings
    configureButtonBindings();
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(m_TeleopSwerve);
    m_fieldSim.initSim();
    // m_ExtendIntakeMotorSubsystem.setDefaultCommand(m_RetractIntakeCommand);
    // m_PitchIntakeSubsystem.setDefaultCommand(new PitchIntakeCommand(m_PitchIntakeSubsystem, 0));
    //stow arm position as default

    // Sets the minimum and maximum pressure for the pneumatics system
    // When the system is beneath the minimum psi, the compressor turns on
    // When the system is above the maximum psi, the compressor turns off
    compressor.enableAnalog(100, 115); // TODO: get values
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
 
  private void configureButtonBindings() {

    SmartDashboard.putBoolean(this.getClass().getSimpleName()+"/DriverController", DriverStation.getJoystickIsXbox(0));
    SmartDashboard.putBoolean(this.getClass().getSimpleName()+"/Co-DriverController", DriverStation.getJoystickIsXbox(1));
    
    //Final Button Bindings
    //--DRIVER CONTROLLER--//
    //--CODRIVER CONTROLLER--//

    // Slow drive
    m_driverController.rightTrigger().onTrue(new InstantCommand(() -> m_TeleopSwerve.toggleSpeed()));
  
    //Lock wheels in an X position
    m_driverController.a().toggleOnTrue(m_SwerveLockCommand);

    //Ground Intake
    m_driverController.leftTrigger()
      .onTrue(m_groundIntakeAndSlowDownCommand)
      .onFalse(m_stowArmCommand);

    //Scoring
    m_driverController.y()
    .onTrue(
      new ConditionalCommand(m_createScoreHighCommand, 
        new ConditionalCommand(
          m_createScoreMediumCommand, 
          m_createScoreLowCommand, 
          m_targetScorePositionUtility::isMedium), 
        m_targetScorePositionUtility::isHigh)
    )
    .and(m_driverController.leftTrigger().negate())
    .onFalse(m_stowArmCommand);

    // Slapstick
    m_codriverController.rightTrigger()
      .onTrue(new SlapstickCommand(true, m_slapstickSubsystem))
      .onFalse(new SlapstickCommand(false, m_slapstickSubsystem));

    m_codriverController.leftTrigger()
      .onTrue(CommandFactoryUtility.createMaxSpeedCommand(m_manipulatorSubsystem))
      .onFalse(CommandFactoryUtility.createStopSpeedCommand(m_manipulatorSubsystem));

    //Scoring Positions
    m_codriverController.povUp().toggleOnTrue(m_targetScorePositionUtility.setDesiredTargetCommand(Target.high));
    m_codriverController.povLeft().toggleOnTrue(m_targetScorePositionUtility.setDesiredTargetCommand(Target.medium));
    m_codriverController.povRight().toggleOnTrue(m_targetScorePositionUtility.setDesiredTargetCommand(Target.medium));
    m_codriverController.povDown().toggleOnTrue(m_targetScorePositionUtility.setDesiredTargetCommand(Target.low));

    // m_driverController.rightTrigger()
    //   .onTrue(new SlapstickCommand(true, m_slapstickSubsystem)
    //     .andThen(new WaitCommand(0.5))
    //     .andThen(CommandFactoryUtility.createMaxSpeedCommand(m_manipulatorSubsystem)))
    //   .onFalse(new SlapstickCommand(false, m_slapstickSubsystem)
    //     .andThen(CommandFactoryUtility.createHoldSpeedCommand(m_manipulatorSubsystem)));
  }

  void checkDSUpdate() {
    Optional <Alliance> currentAllianceOpt = DriverStation.getAlliance();
    Alliance currentAlliance = currentAllianceOpt.isPresent()?currentAllianceOpt.get():null;

    // If we have data, and have a new alliance from last time
    if (DriverStation.isDSAttached() && currentAlliance != null && currentAlliance != alliance) {
      // m_robotDrive.setOriginBasedOnAlliance(); no longer used because of April Tags
      alliance = currentAlliance;
      allianceAssigned = true;
    }

    // Logger.getInstance().recordOutput(this.getClass().getSimpleName()+"/currentAlliance", currentAlliance.toString());
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    checkDSUpdate();
    
    // If not FMS controlled add to teleop init too (for practice match and Red/Blue alliance need to be correctly set)
    return m_autoManager.getAutonomousCommand();
    //TODO determine if autoManager needs to have andThen(() -> m_robotDrive.drive(0, 0, 0, false,false));
  }

  /**
   * Method to run before teleop starts, needed to help reset April Tag direction before teleop if operator does not do 
   * autonomous first.
   */
  public void teleopInit() {
    // If not FMS controlled add to teleop init too (for practice match and Red/Blue alliance need to be correctly set)
    if(!DriverStation.isFMSAttached()) {
      // m_robotDrive.setOriginBasedOnAlliance(); No longer using since April Tags are no longer being used
    }
    
    refollowAllMotors();

  }

  public void periodic() {
    checkDSUpdate();

    m_fieldSim.periodic();
    m_mechanismSimulator.periodic();

    // TODO: Fix, This crashes code
    // if(m_timeOfFlight.sensorDetected()){
    //   m_CurrentPitchIntakeCommand = m_LowPitchIntakeCommand;
    // }
    // else{
    //   m_CurrentPitchIntakeCommand = m_HighPitchIntakeCommand;
    // }
  }

  public void disabledInit() {
  }

  public void testInit() {
    stopSubsystems();
    refollowAllMotors();
  }

  public void testPeriodic() {
    
    /*
     * Code for testing slapstick and top piston
     * 
    // Moves slapstick when left bumper is pressed.
    if (m_driverController.leftBumper().getAsBoolean()) {
      m_slapstickSubsystem.setState(true);
    } else{
      m_slapstickSubsystem.setState(false);
    }

    // Moves top piston when right bumper is pressed.
    if(m_driverController.rightBumper().getAsBoolean()){
      m_topRollerSubsystem.setPistonState(true); 
    }else{
      m_topRollerSubsystem.setPistonState(false);
    }
    */

  }

  public void testExit() {
    refollowAllMotors();
  }

  private void refollowAllMotors() {
    m_manipulatorSubsystem.refollowMotors();
  }

  private void stopSubsystems() {
    m_manipulatorSubsystem.setRollerSpeed(0.0);
  }

}