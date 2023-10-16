package frc.robot.subsystems.manipulator;

// TODO document interface
public interface ManipulatorIO {
    
    public void updateInputs();
    public double getRollerVoltage();
    public void setRollerSpeed(double speed);
    public double getRollerCurrent();
    public void refollow();
}