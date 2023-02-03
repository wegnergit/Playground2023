package frc.robot.autos;

import java.util.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.*;

/**
 * <h3>AutonomouseCommandManager</h3>
 * 
 * Manages the autonomous paths by creating an instance of them and putting them
 * into the Shuffleboard.
 */
public class AutoCommandManager {
    HashMap<String, Subsystem> subsystemMap = new HashMap<String, Subsystem>();
    private HashMap<String, Command> m_eventCommandMap;

    public static enum subNames {
        SwerveDriveSubsystem("SwerveDrive");

        final String m_name;

        subNames(String name) {
            m_name = name;
        }
    }

    public AutoCommandManager(HashMap<String, Command> eventCommandMap) {
        m_eventCommandMap = eventCommandMap;
    }
    /**
     * Adds a subbsystem to the subystem map
     *
     * @param SubNames
     * @param subsbystem
     */
    public void addSubsystem(subNames SubNames, Subsystem subsystem) {
        subsystemMap.put(SubNames.toString(), subsystem);
    }

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    /**
     * Creates instances of each autonomous path command
     */
    public void initCommands() {
        // Subsystems needed
        SwerveDrive swerveDrive = (SwerveDrive) subsystemMap.get(subNames.SwerveDriveSubsystem.toString());

        // Commands
        TaxiOneBall taxiOneBall = new TaxiOneBall(swerveDrive, false);
        Command firstCone = new FirstCone(swerveDrive, m_eventCommandMap, false);
        Command firstConeCommand = new PathPlannerCommand("FirstCone", swerveDrive, m_eventCommandMap, false);
        TaxiOneBall taxiOneBallWithPoseEst = new TaxiOneBall(swerveDrive, true);
        Command firstConeWithPoseEst = new FirstCone(swerveDrive, m_eventCommandMap, true);
        Command firstConeCommandWithPoseEst = new PathPlannerCommand("FirstCone", swerveDrive, m_eventCommandMap, true);

        // Add options to chooser widget
        m_chooser.setDefaultOption("None", null);
        m_chooser.addOption("Taxi One Ball", taxiOneBall);
        m_chooser.addOption("FirstCone", firstCone);
        m_chooser.addOption("FirstConeCommand", firstConeCommand);
        m_chooser.addOption("Taxi One Ball W Pose Est", taxiOneBallWithPoseEst);
        m_chooser.addOption("FirstCone W Pose Est", firstConeWithPoseEst);
        m_chooser.addOption("FirstConeCommand W Pose Est", firstConeCommandWithPoseEst);

        // Place Chooser on dashboard
        SmartDashboard.putData("Auto choices", m_chooser);
    }
    /**
     *
     * Gets the autonomous path that is selected in the Shuffleboard
     *
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

}
