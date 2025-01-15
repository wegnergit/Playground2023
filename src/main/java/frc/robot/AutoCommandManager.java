package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.PukerSubsystem;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommandManager {

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoCommandManager(Drive drive, PukerSubsystem puker) {
    configureNamedCommands(puker, drive);
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    PathPlannerAuto MidScore1 = new PathPlannerAuto("MidScore1");
    PathPlannerAuto MidScore2 = new PathPlannerAuto("MidScore2");
    PathPlannerAuto Circle = new PathPlannerAuto("Circle");
    PathPlannerAuto Straight = new PathPlannerAuto("Straight");

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("MidScore2", MidScore2);
    autoChooser.addOption("MidScore1", MidScore1);
    autoChooser.addOption("Circle", Circle);
    autoChooser.addOption("Straight", Straight);
  }

  public void configureNamedCommands(PukerSubsystem puker, Drive drivetrain) {
    NamedCommands.registerCommand(
        "troughScoreCommand",
        puker
            .newStartMotorCommand()
            .alongWith(new WaitCommand(1))
            .andThen(puker.newStopMotorCommand()));
    NamedCommands.registerCommand("stopDrive", DriveCommands.brakeDrive(drivetrain));
    NamedCommands.registerCommand(
        "printA", new PrintCommand("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%AAAAAAAAAAA"));
    NamedCommands.registerCommand(
        "printB", new PrintCommand("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BBBBBBB"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
