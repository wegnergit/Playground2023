package frc.robot.utilities;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.armcommands.RunManipulatorRollerCommand;
import frc.robot.commands.armcommands.RunTopRollerCommand;
import frc.robot.commands.armcommands.SetArmDegreesCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;

public class CommandFactoryUtility {
    //Cube Ground Intake
    public static final double ARM_INTAKE_ANGLE = 0.0;

    //Stow Position
    public static double STOW_POSITION = 135.0;

    //Low Score
    public static final double ARM_LOW_SCORE_ANGLE = STOW_POSITION; //STOW_POSITION;
    
    //Medium Score
    public static final double ARM_MEDIUM_SCORE_ANGLE = 125.0;

    //High Score
    public static final double ARM_HIGH_SCORE_ANGLE = 120.0;



    private CommandFactoryUtility() {}

    public static Command createIntakeCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        TopRollerSubsystem m_topRollerSubsystem) {
        final Command command = 
            new SetArmDegreesCommand(m_armSubsystem, ARM_INTAKE_ANGLE)
                .andThen(new RunTopRollerCommand(m_topRollerSubsystem, TopRollerSubsystem.ROLLER_INTAKE_SPEED, true))
                .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.ROLLER_INTAKE_SPEED));
        return command;
    }

    public static Command createIntakeAndSlowDownCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        TopRollerSubsystem m_topRollerSubsystem, 
        TeleopSwerve m_TeleopSwerve) {
        final Command command = 
                createIntakeCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem)
                .andThen(m_armSubsystem.createWaitUntilAtAngleCommand().withTimeout(1.0))
                .andThen(new InstantCommand(() -> m_TeleopSwerve.forceSlowSpeed()))

                .andThen(m_manipulatorSubsystem.waitUntilCurrentPast(17.0))
                .andThen(createStowArmAndSpeedUpCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem, m_TeleopSwerve));
        return command;
    }

    public static Command createStowArmCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        TopRollerSubsystem m_topRollerSubsystem) {
        final Command command = 
            new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.HOLD_SPEED)
            .andThen(new RunTopRollerCommand(m_topRollerSubsystem, 0.0, false))
            .andThen(new SetArmDegreesCommand(m_armSubsystem, STOW_POSITION))
            .andThen(m_armSubsystem.createWaitUntilLessThanAngleCommand(170.0))    
            .andThen(m_armSubsystem.createWaitUntilGreaterThanAngleCommand(45.0));
        return command;
    }

    public static Command createStowArmAndSpeedUpCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        TopRollerSubsystem m_topRollerSubsystem, 
        TeleopSwerve m_TeleopSwerve) {
        final Command command = 
            new InstantCommand(() -> m_TeleopSwerve.forceNormalSpeed())    
            .andThen(createStowArmCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem));
            
        return command;
    }

    public static Command createScoreCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        double m_armPosition,
        double m_manipulatorspeed,
        double m_timeOut) {
        Command command;

        command = 
            new SetArmDegreesCommand(m_armSubsystem, m_armPosition)
            .andThen(m_armSubsystem.createWaitUntilAtAngleCommand().withTimeout(m_timeOut))
            .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, m_manipulatorspeed));
        return command;
    }

    public static Command createScoreLowCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {

        return createScoreCommand(m_armSubsystem, m_manipulatorSubsystem, ARM_LOW_SCORE_ANGLE, ManipulatorSubsystem.LOW_SCORE_SPEED, 0.5);
    }

    public static Command createScoreMediumCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {

        return createScoreCommand(m_armSubsystem, m_manipulatorSubsystem, ARM_MEDIUM_SCORE_ANGLE, ManipulatorSubsystem.MEDIUM_SCORE_SPEED, 0.5);
    }

    public static Command createScoreHighCommand(
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {

        return createScoreCommand(m_armSubsystem, m_manipulatorSubsystem, ARM_HIGH_SCORE_ANGLE, ManipulatorSubsystem.HIGH_SCORE_SPEED, 0.5);
    }

    public static Command createMaxSpeedCommand(ManipulatorSubsystem m_manipulatorSubsystem) {
        Command command;

        command = new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.MAX_SPEED);
        return command;
    }

    public static Command createHoldSpeedCommand(ManipulatorSubsystem m_ManipulatorSubsystem) {
        Command command;

        command = new RunManipulatorRollerCommand(m_ManipulatorSubsystem, ManipulatorSubsystem.HOLD_SPEED);
        return command;
    }

    public static Command createStopSpeedCommand(ManipulatorSubsystem m_ManipulatorSubsystem) {
        Command command;

        command = new RunManipulatorRollerCommand(m_ManipulatorSubsystem, 0.0);
        return command;
    }

    /**
     * addAutoCommandEvent
     *  Add event command to event map for autonomous paths
     * 
     * @param eventCommandMap
     * @param eventName
     * @param m_armSubsystem
     * @param m_manipulatorSubsystem
     */
    public static void addAutoCommandEvent( Map<String, Command> eventCommandMap, 
                                            String eventName, 
                                            ArmSubsystem m_armSubsystem, 
                                            ManipulatorSubsystem m_manipulatorSubsystem,
                                            TopRollerSubsystem m_topRollerSubsystem) {
        Command autoCommand = null;
        switch(eventName) {
            case "intakeCube":
                autoCommand = CommandFactoryUtility.createIntakeCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem)
                    .andThen(new WaitCommand(1.0));
                break;
            case "stowArm":
                autoCommand = CommandFactoryUtility.createStowArmCommand(m_armSubsystem, m_manipulatorSubsystem, m_topRollerSubsystem);
                break;
            case "shootCube":
                autoCommand = CommandFactoryUtility.createMaxSpeedCommand(m_manipulatorSubsystem)
                .andThen(new WaitCommand(1.0))
                .andThen(CommandFactoryUtility.createHoldSpeedCommand(m_manipulatorSubsystem));
                break;
        }

        if(autoCommand != null) {
            eventCommandMap.put(eventName, autoCommand);
        } else {
            DriverStation.reportWarning("Unable to add event name"+eventName+" given not declared!", 
                Thread.currentThread().getStackTrace());
        }
     
    }

}