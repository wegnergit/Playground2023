package frc.robot.autos;

import java.util.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.PPSwerveControllerCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;
import frc.robot.utilities.CommandFactoryUtility;

/**
 * <h3>AutonomouseCommandManager</h3>
 * 
 * Manages the autonomous paths by creating an instance of them and putting them
 * into the Shuffleboard.
 */
public class AutoCommandManager {
    public AutoCommandManager() {
        SmartDashboard.putBoolean(this.getClass().getSimpleName()+"/CanTunePIDValues", TUNE_PID);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kPX", kPXController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kIX", kIXController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kDX", kDXController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kPY", kPYController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kIY", kIYController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kDY", kDYController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kPTheta", kPThetaController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kITheta", kIThetaController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kDTheta", kDThetaController);
    }

    HashMap<String, Subsystem> subsystemMap = new HashMap<String, Subsystem>();
    private Field2d pp_field2d = new Field2d();

    public static enum subNames {
        SwerveDriveSubsystem("SwerveDrive"),
        // ElevatorSubsystem("Elevator"),
        ArmSubsystem("Arm"),
        ManipulatorSubsystem("Manipulator"),
        TopRollerSubsystem("TopRoller");

        final String m_name;

        subNames(String name) {
            m_name = name;
        }
    }

    /**
     * Adds a subsystem to the subystem map
     *
     * @param SubNames
     * @param subsystem
     */
    public void addSubsystem(subNames SubNames, Subsystem subsystem) {
        subsystemMap.put(SubNames.toString(), subsystem);
    }

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    private static final boolean TUNE_PID = true;
    //TODO TUNE FOR GHOST
    public static final double kPXController = usePIDValueOrTune("kPX",3.596); //0.076301;
    public static final double kIXController = usePIDValueOrTune("kIX",0.0);; 
    public static final double kDXController = usePIDValueOrTune("kDX",0.0);; 
    public static final double kPYController = usePIDValueOrTune("kPY",3.596); //0.076301;
    public static final double kIYController = usePIDValueOrTune("kIY",0.0); 
    public static final double kDYController = usePIDValueOrTune("kDY",0.0);  
    public static final double kPThetaController = usePIDValueOrTune("kPTheta",3.2);
    public static final double kIThetaController = usePIDValueOrTune("kITheta",0.0);
    public static final double kDThetaController = usePIDValueOrTune("kDTheta",0.0);
    
    /**
     * <h3>initCommands</h3>
     * 
     * Creates instances of each autonomous path command
     * 
     * @param eventCommandMap a command that uses strings to returna command that we want to execute at a marker
     */
    public void initCommands(Map<String, Command> eventCommandMap) {
        // Setup Logging for PathPlanner (for Ghost image)
        // NOTE: Make sure same as setting correct PPSwerveControllerCommand (ours or theirs)
        PPSwerveControllerCommand.setLoggingCallbacks(
                null, 

                (Pose2d targetPose) -> {
                    // Log target pose
                    pp_field2d.setRobotPose(targetPose);
                    // May just want dashboard not on field2d
                    Logger.getInstance().recordOutput("PathPlanner/DesiredPose",targetPose);
                },
                null, // logSetPoint

                null // loggError

        );
        SmartDashboard.putData("PP_Field", pp_field2d);
        //Subsystems used by auto commands
        SwerveDrive s_SwerveDrive = (SwerveDrive) subsystemMap.get(subNames.SwerveDriveSubsystem.toString());
        //ElevatorSubsystem m_elevatorSubsystem = (ElevatorSubsystem) subsystemMap.get(subNames.ElevatorSubsystem.toString());
        ArmSubsystem m_armSubsystem = (ArmSubsystem) subsystemMap.get(subNames.ArmSubsystem.toString());
        ManipulatorSubsystem m_manipulatorSubsystem = (ManipulatorSubsystem) subsystemMap.get(subNames.ManipulatorSubsystem.toString());
        TopRollerSubsystem m_topRollerSubsystem = (TopRollerSubsystem) subsystemMap.get(subNames.TopRollerSubsystem.toString());
        
        //Autonomous Commands
        Command HighScoreBumpMobilityCommand = new PathPlannerCommand(
            CommandFactoryUtility.createScoreHighCommand(m_armSubsystem, m_manipulatorSubsystem)
            .andThen(new WaitCommand(1.0))
            .andThen(CommandFactoryUtility.createStowArmCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem)),
            s_SwerveDrive, "HighScoreBumpMobility_u", eventCommandMap
        );

        Command TwoScoreBumpCommand = new PathPlannerCommand(
            CommandFactoryUtility.createScoreHighCommand(m_armSubsystem, m_manipulatorSubsystem)
            .andThen(new WaitCommand(1.0))
            .andThen(CommandFactoryUtility.createStowArmCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem)),
            s_SwerveDrive, "TwoScoreBump_uu", eventCommandMap
        );

        Command TwoScoreAndPickupBumpCommand = new PathPlannerCommand(
            CommandFactoryUtility.createScoreHighCommand(m_armSubsystem, m_manipulatorSubsystem)
            .andThen(new WaitCommand(1.0))
            .andThen(CommandFactoryUtility.createStowArmCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem)),
            s_SwerveDrive, "TwoScoreAndPickupBump_uuu", eventCommandMap
        );


        // Adding options to the chooser in Shuffleboard/smartdashboard
        Boolean isBlue = (DriverStation.getAlliance() == Alliance.Blue);
        m_chooser.setDefaultOption("None", null);

        m_chooser.addOption("HighScoreBumpMobility_u", HighScoreBumpMobilityCommand);
        m_chooser.addOption("TwoScoreBump_uu", TwoScoreBumpCommand);
        m_chooser.addOption("TwoScoreAndPickupBump_uuu", TwoScoreAndPickupBumpCommand);

        //Adding chooser to Shuffleboard/Smartdashboard
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    /**
    * <h3>usePIDValueOrTune</h3>
    *
    * manually set a defaultValue in robot container and if TUNE_PID is true 
    * it gives pidValue with the key and defaultValue if false gives just the defaultValue
    * @param key
    * @param defaultValue
    */
    public static double usePIDValueOrTune(String key, double defaultValue) {
        double pidValue;
        if(TUNE_PID) {
            pidValue = Preferences.getDouble(key, defaultValue);
        } else {
            pidValue = defaultValue;
        }
        // Save Value so don't need tuning (so value is saved)
        Preferences.setDouble(key, pidValue);     
        return pidValue;
    }
    
    /**
     *<h3> getAutonomousCommand</h3>

     * Gets the autonomous path that is selected in the Shuffleboard
     *
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}