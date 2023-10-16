package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utilities.SparkMaxWrapper;

public class ManipulatorIOSim implements ManipulatorIO {

    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNEO(1), 75,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(9.4), Units.lbsToKilograms(4)),
            Units.inchesToMeters(9.4), -2 * Math.PI, 2 * Math.PI, true);
    private final SparkMaxWrapper roller = new SparkMaxWrapper(15, MotorType.kBrushless);

    @Override
    public void refollow() {}

      /**
     * <h3>updateInputs</h3>
     */
    @Override
    public void updateInputs() {
        sim.update(0.02);
    }
    /**
     * <h3>getRollaerVoltagee
     * <h3>
     * gets the roller voltage
     * 
     * @return getBusVoltage
     */
    @Override
    public double getRollerVoltage() {
        return roller.getBusVoltage();
    }
    /**
     * <h3>setRollerSpeed
     * <h3>
     * sets the speed for the roller
     * 
     * @param speed
     */
    @Override
    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    @Override
    public double getRollerCurrent() {
        return roller.getOutputCurrent();
    }
}
