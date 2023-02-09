package frc.robot.commands;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.I2CPortLEDSubSystem;

public class LEDOnArduinoUsingI2C extends CommandBase {
  
    public static enum ledActions {
        CONE,
        CUBE,
        CONE_ON_SHELF, CUBE_ON_STAND
    }
    private ledActions m_action;
    private I2CPortLEDSubSystem m_i2cSubSystem;

    // Open a new I2C connection on port 4  (TODO create subsystem for this)
    

    public LEDOnArduinoUsingI2C(I2CPortLEDSubSystem i2cPortLEDSubSystem,  ledActions action) {
        m_action = action; 
        m_i2cSubSystem = i2cPortLEDSubSystem;
        addRequirements(m_i2cSubSystem);
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {
        switch(m_action) {
            case CONE:
                m_i2cSubSystem.sendInfo( "CONE");
                break;
            case CUBE:
                m_i2cSubSystem.sendInfo( "CUBE");
                break;
            case CUBE_ON_STAND:
                m_i2cSubSystem.sendInfo( "CUBE_N_STAND");
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
