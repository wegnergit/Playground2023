package frc.robot.autos;

import frc.robot.commands.PPSwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.SwerveAutoBuilder;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathPlannerCommand extends SequentialCommandGroup {
    public PathPlannerCommand(String pathName, SwerveDrive s_Swerve, Map<String, Command> eventMap, boolean usePoseEst) {
        addRequirements(s_Swerve); // TODO WAS MISSING THIS

        // Load from path planner (or TODO  do you want to pass in)
        PathConstraints pathConstraints = PathPlanner.getConstraintsFromPath(pathName);
        // If did not set in path planner default values
        if(pathConstraints == null) {
            // TODO use CONSTANTS
            pathConstraints = new PathConstraints(1, 2.5);          
        }
        // An example trajectory to follow.  All units in meters.
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, 
            false,
            pathConstraints);
        
        PIDController thetaController = s_Swerve.getAutoThetaController();
        thetaController.enableContinuousInput(-180.0, 180.0); //-Math.PI, Math.PI);

        SwerveAutoBuilder autoBuilder = 
            new SwerveAutoBuilder(
                // TODO originally use getPoseMeters
                (usePoseEst)?s_Swerve::getPose:s_Swerve::getPoseMeters,
                // TODO is it using reset and should it use pose estimator
                s_Swerve::resetOdometry,
                SwerveDrive.getSwerveKinematics(),
                // TODO it use constants not PID controller and only one not one for x and y
                getPIDConstants(s_Swerve.getAutoXController()),
                // getPIDConstants(s_Swerve.getAutoYController()),
                getPIDConstants(thetaController),
                s_Swerve::setSwerveModuleStatesDuringAuto,
                eventMap,
                // TODO validate works if red side.  else pass false and create new paths for red side.
                // May want to have seperate anyhow just in case red side different from blue at event
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true    
                s_Swerve);

        Command swerveControllerCommand = autoBuilder.fullAuto(pathGroup);
        addCommands(
            // new InstantCommand(() -> s_Swerve.resetOdometry(pathPlannerExample.getInitialHolonomicPose())),
            swerveControllerCommand
        );
    }

    private PIDConstants getPIDConstants(PIDController pidController) {
        return new PIDConstants(pidController.getP(), pidController.getI(), pidController.getD());
    }
}