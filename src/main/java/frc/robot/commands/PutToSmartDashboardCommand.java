package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PutToSmartDashboardCommand extends Command {
    public PutToSmartDashboardCommand(String key, Boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public PutToSmartDashboardCommand(String key, Double value) {
        SmartDashboard.putNumber(key, value);
    }

}
