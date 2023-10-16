package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.utilities.SparkMaxWrapper;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class ManipulatorIORobot implements ManipulatorIO { 

    // -------- DECLARATIONS --------\\
    private final CANSparkMax leaderRoller;
    private final CANSparkMax followerRoller;

    // -------- CONSTANTS --------\\
    // Constant, in amps
    private final int STALL_LIMIT = 10;
    private final int FREE_LIMIT = 20;

    // TRY to keep offset value away from flipping around to 359 it would flip around in side
    public final static double SAFE_ZONE_OFFSET = 85;

    //----------Constructor---------\\
    public ManipulatorIORobot(int leaderRollerID, int followerRollerID) {
        leaderRoller = new SparkMaxWrapper(leaderRollerID, MotorType.kBrushless);
        followerRoller = new SparkMaxWrapper(followerRollerID, MotorType.kBrushless);

        leaderRoller.restoreFactoryDefaults();
        leaderRoller.setIdleMode(IdleMode.kCoast);

        followerRoller.restoreFactoryDefaults();
        followerRoller.setIdleMode(IdleMode.kCoast);
        
        refollow();

        // TODO: Determine if this helps encoder position update faster
        leaderRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        leaderRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        // TODO: Was too low when tested
        //roller.setSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT);

        leaderRoller.setInverted(true);
    }
    @Override
    public void refollow() {
        followerRoller.follow(leaderRoller, true);
    }
     /**
     * <h3>updateInputs</h3>
     * 
     * Left blank because it's only used in simulation
     */
    @Override
    public void updateInputs() {}
     /**
     * <h3>getRollerVoltage</h3>
     * Returns the current voltage of the roller
     * @return roller.getBusVoltage()
     */
    @Override
    public double getRollerVoltage() {
        return leaderRoller.getBusVoltage();
    }
     /**
     * <h3>setRollerSpeed</h3>
     * Set the speed of the roller
     * @param speed
     */
    @Override
    public void setRollerSpeed(double speed) {
        leaderRoller.set(speed);
    }
    @Override
    public double getRollerCurrent() {
        return leaderRoller.getOutputCurrent();
    }
}
