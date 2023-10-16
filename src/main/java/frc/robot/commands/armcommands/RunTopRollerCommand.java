package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TopRollerSubsystem;

public class RunTopRollerCommand extends Command{
    
    private final double m_rollerSpeed;
    private final boolean m_pistonOpen;

    private final TopRollerSubsystem m_topRoller;

    public RunTopRollerCommand(TopRollerSubsystem topRollerSubsystem, double speed, boolean pistonOpen) {
        m_topRoller = topRollerSubsystem;
        m_rollerSpeed = speed;
        m_pistonOpen = pistonOpen;

        addRequirements(topRollerSubsystem);
    }

    @Override
    public void initialize() {
       m_topRoller.setRollerSpeed(m_rollerSpeed);
       m_topRoller.setPistonState(m_pistonOpen);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
