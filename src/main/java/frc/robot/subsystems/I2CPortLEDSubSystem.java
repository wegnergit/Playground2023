package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class I2CPortLEDSubSystem extends SubsystemBase {
    private I2C m_wire;
    private int m_deviceAddress;

    public I2CPortLEDSubSystem(int deviceAddress) {
        m_deviceAddress = deviceAddress;
        m_wire = new I2C(Port.kOnboard, m_deviceAddress);
    }

    public  void sendInfo(String text){
        char[] CharArray = text.toCharArray(); // Turn the string into a character array
        
        byte[] WriteData = new byte[CharArray.length]; //make a new array to fill with bytes
        for(int i = 0; i < CharArray.length; i++){
            WriteData[i] = (byte)CharArray[i]; //turn each char into bytes and add it to the byte array
        }
    
        m_wire.transaction(WriteData, WriteData.length, null, 0); //send byte array to device
    }


}
