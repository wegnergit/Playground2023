package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.SparkMaxWrapper;

public class TopRollerSubsystem extends SubsystemBase{

    private final CANSparkMax topRoller;
    private final Solenoid rollerPiston;

    public static final double ROLLER_INTAKE_SPEED = 0.4;

    public TopRollerSubsystem(int topRollerID, int solenoidID) {
        topRoller = new SparkMaxWrapper(topRollerID, MotorType.kBrushless);

        topRoller.restoreFactoryDefaults();
        topRoller.setIdleMode(IdleMode.kBrake);
        
        topRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        topRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        topRoller.setInverted(true);

        //CTRE pneumatic hub has 8 slots. Cap is placed on simulation to prevent errors.
        rollerPiston = new Solenoid(
            Robot.isReal() || solenoidID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM, 
            solenoidID);
    }

    /**<h3>getRollerSpeed</h3>
     * Sets the top roller speed
     * @return setRollerSpeed
     */
    public void setRollerSpeed(double speed) {
        topRoller.set(speed);
        Logger.getInstance().recordOutput(this.getClass().getSimpleName() + "/TopRollerspeed", speed);
    }

    //-- PISTON --\\
    public void togglePiston() {
        rollerPiston.set(!rollerPiston.get());
        Logger.getInstance().recordOutput(this.getClass().getSimpleName() + "/pistonState", rollerPiston.get());
    }
    
    public boolean pistonIsOpen() {
        return rollerPiston.get();
    }

    public void setPistonState(boolean open) {
        rollerPiston.set(open);
        Logger.getInstance().recordOutput(this.getClass().getSimpleName() + "/pistonState", rollerPiston.get());
    }
}
