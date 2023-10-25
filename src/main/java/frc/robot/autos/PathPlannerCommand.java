package frc.robot.autos;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.FieldCentricOffset;
// import frc.robot.utilities.SwerveAutoBuilder;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*
 * <h3>PathPlannerCommand<h3>
 * 
 */
public class PathPlannerCommand extends SequentialCommandGroup {

    private static final double MAX_ACCELERATION = 2.5;
    private static final double MAX_VELOCITY = 1.0;

    /**
     * <h3>PathPlannerCommand</h3>
     * 
     * adding path constraints and builds auto command
     * 
     * @param preCommand a command that is run before the path starts
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to return a command that we want to execute at a marker
     * @param postCommand a command that is run after the path completes
     */
    public PathPlannerCommand(Command preCommand, SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap, Command postCommand) {
        addRequirements(s_Swerve);

        // Adding a pre command to autonomous ex. autoBalance
        if(preCommand != null) {
            addCommands(preCommand);
        }

        // TODO May not need given autoconfigure has a resetPose???
        //https://github.com/mjansen4857/pathplanner/wiki/Java-Example:-Path-Groups
        Pose2d startingPose = getStartingPose(pathName);
        
        
        // creates a command based on the path group
        Command swerveControllerCommand = getAutoCommand(pathName);

        addCommands(
            // TODO: Use april tags to help set this
            new InstantCommand(() -> FieldCentricOffset.getInstance().setOffset(startingPose.getRotation().getDegrees())),
            swerveControllerCommand
        );
        if (postCommand != null) {
            addCommands(postCommand);
        }
    }

    private Command getAutoBuilderFollowCommand(String pathName) {
        // //https://github.com/mjansen4857/pathplanner/wiki/Java-Example:-Follow-a-Single-Path
        // // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        // // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPathWithEvents(path);
    }

    private PathPlannerAuto getPathPlannerAutoCommand(String pathName) {
        // creates a command based on the auto (NO EVENTS though)
        //https://github.com/mjansen4857/pathplanner/wiki/Java-Example:-Path-Groups
        return new PathPlannerAuto(pathName);
    }
    
    private Command getAutoCommand(String pathName) {
        // None seem to execute EventMarkers
        if (false) {
            // no starting point
            return getAutoBuilderFollowCommand(pathName);
        } else {
            // has starting point
            return getPathPlannerAutoCommand(pathName);
        }
    }

    private Pose2d getStartingPose(String pathName) {
        double degrees;
        Pose2d startingPose;
        try {
            //https://github.com/mjansen4857/pathplanner/wiki/Java-Example:-Path-Groups
            startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(pathName);
            degrees = startingPose.getRotation().getDegrees();
        } catch ( RuntimeException ex) {
            startingPose = new Pose2d(0.0,0.0,new Rotation2d(0.0, 0.0));
            degrees = 0.0;
        }
        return startingPose;
    }

    /**
     * 
     * <h3>PathPlannerCommand</h3>
     * 
     * Adds path constraints and builds auto command
     * 
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to return a command that we want to execute at a marker
     * @param postCommand a command that is run after the path completes
     */
    public PathPlannerCommand(SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap, Command postCommand) {
            this(null, s_Swerve, pathName, eventCommandMap, postCommand);
    }


    /**
     * <h3>PathPlannerCommand</h3>
     * 
     * Adds path constraints and builds auto command
     * 
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to return a command that we want to execute at a marker
     */
    public PathPlannerCommand(SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap) {
        this(s_Swerve, pathName, eventCommandMap, null);
    }

    /**
     * 
     * <h3>PathPlannerCommand</h3>
     * 
     * Adds path constraints and builds auto command
     * 
     * @param preCommand a command that is run before the path starts
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to return a command that we want to execute at a marker
     */
    public PathPlannerCommand(Command preCommand, SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap) {
        this(preCommand, s_Swerve, pathName, eventCommandMap, null);
    }
}