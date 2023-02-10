package frc.robot.commands;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DIOPortsForLEDSSubSystem;
import frc.robot.subsystems.I2CPortLEDSubSystem;

public class LEDOnArduinoUsingDIO extends CommandBase {
  
    public static enum ledActions {
        CONE,
        CUBE,
        CONE_ON_SHELF, CUBE_ON_STAND
    }
    private ledActions m_action;
    private DIOPortsForLEDSSubSystem m_dioPostSubSystem;


    public LEDOnArduinoUsingDIO(DIOPortsForLEDSSubSystem dioPortLEDSubSystem,  ledActions action) {
        m_action = action; 
        m_dioPostSubSystem = dioPortLEDSubSystem;
        addRequirements(m_dioPostSubSystem);
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void initialize() {
        switch(m_action) {
            case CONE:
                m_dioPostSubSystem.setPins(true,false,false,false,false);
                break;
            case CUBE:
                m_dioPostSubSystem.setPins(false,true,false,false,false);
                break;
            case CUBE_ON_STAND:
                m_dioPostSubSystem.setPins(false,false,true,false,false);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
