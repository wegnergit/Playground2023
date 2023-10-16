package frc.robot.subsystems;

// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class SlapstickSubsystem extends SubsystemBase{

    private final Solenoid m_slapstick;

    public SlapstickSubsystem(int solenoid_ID) {
        
        //CTRE pneumatic hub has 8 slots. Cap is placed on simulation to prevent errors.
        m_slapstick = new Solenoid(
            Robot.isReal() || solenoid_ID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM, 
            solenoid_ID);
    }

    public void togglePiston() {
        m_slapstick.set(!m_slapstick.get());
        // Logger.getInstance().recordOutput(this.getClass().getSimpleName() + "/pistonState", m_slapstick.get());
    }
    
    public boolean pistonIsOpen() {
        return m_slapstick.get();
    }

    public void setState(boolean open) {
        m_slapstick.set(open);
        // Logger.getInstance().recordOutput(this.getClass().getSimpleName() + "/pistonState", m_slapstick.get());
    }
}