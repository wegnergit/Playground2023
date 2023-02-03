package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.SwerveAutoBuilder;

public class FirstCone  extends SequentialCommandGroup {

    
    public FirstCone(SwerveDrive driveSubsystem, HashMap<String, Command> eventMap, boolean usePoseEst) {
        addRequirements(driveSubsystem);

        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FirstCone", new PathConstraints(4, 3));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // eventMap.put("intakeDown", new InstantCommand( () -> {
        //     System.out.println("TESTING.......");}));
        // eventMap.put("placeCone", new InstantCommand( () -> {
        //     System.out.println("Placing Cone......");}));
    
        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            // TODO TRY Estimator but field2d not displaying
            (usePoseEst)?driveSubsystem::getPose:driveSubsystem::getPoseMeters, //Est, // Pose2d supplier
            driveSubsystem::resetOdometry, //Est, // Pose2d consumer, used to reset odometry at the beginning of auto
            SwerveDrive.getSwerveKinematics(), // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            driveSubsystem::setSwerveModuleStatesDuringAuto, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        Command swerveControllerCommand = autoBuilder.fullAuto(pathGroup);
        addCommands(
            // new InstantCommand(() -> driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
        
    }

}
