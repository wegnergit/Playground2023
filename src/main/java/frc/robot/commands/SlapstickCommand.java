package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SlapstickSubsystem;

public class SlapstickCommand extends Command {
    
    private final boolean open;

    private final SlapstickSubsystem m_piston;

    public SlapstickCommand(boolean pistonOpen, SlapstickSubsystem slapstickSubsystem) {
        open = pistonOpen;
        m_piston = slapstickSubsystem;

        addRequirements(m_piston);
    }

    @Override
    public void initialize() {
        m_piston.setState(open);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
