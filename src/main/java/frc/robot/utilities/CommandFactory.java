package frc.robot.utilities;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.armcommands.RunManipulatorRollerCommand;
import frc.robot.commands.armcommands.SetArmDegreesCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CommandFactory {

    // public static HashMap <String, Command> scoreHighGoal 
    public CommandFactory() {
    }

    /** Score Goal
     * @param m_elevatorSubsystem 
     * @param m_armSubsystem 
     * @param m_manipulatorSubsystem 
     * @param evelatorHeight
     * @param waitSecondAterEvelator 
     * @param armPosition
     * @param waitSecondArm 
     * @param manipulatorPosition
     * @return
     */
    public static Command createScoreCommand (ElevatorSubsystem m_elevatorSubsystem, 
        ArmSubsystem m_armSubsystem, 
        ManipulatorSubsystem m_manipulatorSubsystem, 
        double evelatorHeight,
        double waitSecondAterEvelator, 
        double armPosition,
        double waitSecondArm,
        double manipulatorPosition) { 
         Command command;
        command = new SequentialCommandGroup(
                    new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(evelatorHeight)),
                    new WaitCommand(waitSecondAterEvelator), // TODO get height and waituntil
                    new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, armPosition, manipulatorPosition),
                    new WaitCommand(waitSecondArm),  // TODO get angle and waituntil
                    new RunManipulatorRollerCommand(m_manipulatorSubsystem,  ManipulatorSubsystem.RELEASE_SPEED) 
        );

        command =  new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(evelatorHeight))
                    .andThen(new WaitCommand(waitSecondAterEvelator)) // TODO get height and waituntil
                    .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem,armPosition, manipulatorPosition))
                    .andThen(new WaitCommand(waitSecondArm)) // TODO get angle and waituntil
                    .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem,  ManipulatorSubsystem.RELEASE_SPEED));
        
        return command;
    }

    /** create command to score High goal
     * @param m_elevatorSubsystem 
     * @param m_armSubsystem 
     * @param m_manipulatorSubsystem 
     * @return
     */
    public static Command createScoreHighCommand (ElevatorSubsystem m_elevatorSubsystem, 
        ArmSubsystem m_armSubsystem, 
        ManipulatorSubsystem m_manipulatorSubsystem) { 
            return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
                    55.0, 2.0, 35.0, 1.0, 0.0);
    }

    /** create command to score medium goal
     * @param m_elevatorSubsystem 
     * @param m_armSubsystem 
     * @param m_manipulatorSubsystem 
     * @return
     */
    public static Command createScoreMediumCommand (ElevatorSubsystem m_elevatorSubsystem, 
        ArmSubsystem m_armSubsystem, 
        ManipulatorSubsystem m_manipulatorSubsystem) { 
            return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
                    22.0, 1.0, 35.0, 1.0, 0.0);
    }

    /** create command to score low goal
     * @param m_elevatorSubsystem 
     * @param m_armSubsystem 
     * @param m_manipulatorSubsystem 
     * @return
     */
    public static Command createScoreLowCommand (ElevatorSubsystem m_elevatorSubsystem, 
        ArmSubsystem m_armSubsystem, 
        ManipulatorSubsystem m_manipulatorSubsystem) { 
            return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
                    12.0, 1.0, -25.0, 1.5, 25.0);
    }

    /**
     * Create command to stow arm
     * @param m_elevatorSubsystem
     * @param m_armSubsystem
     * @param m_manipulatorSubsystem
     * @return
     */
    public static Command createStowArmCommand (ElevatorSubsystem m_elevatorSubsystem, 
        ArmSubsystem m_armSubsystem, 
        ManipulatorSubsystem m_manipulatorSubsystem) { 
            final Command command =      new ParallelCommandGroup(
                new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(0)),
                new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, ArmSubsystem.STOW_POSITION, ManipulatorSubsystem.STOW_POSITION),
                new RunManipulatorRollerCommand(m_manipulatorSubsystem, 0.15) // TODO constant
              );
            return command;
    }  

}
