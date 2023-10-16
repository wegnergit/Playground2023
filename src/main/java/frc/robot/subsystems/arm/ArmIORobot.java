package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Preferences;

public class ArmIORobot implements ArmIO {

    private CANSparkMax m_leaderArmMotor;
    private CANSparkMax m_followerArmMotor;
    
    private AbsoluteEncoder m_leaderArmEncoder;

    private static final String PREFERENCE_NAME = "ArmOffsetDegrees";
    private static double m_armOffsetDegrees = Preferences.getDouble(PREFERENCE_NAME, -12.0);

    // 6.3 encoder value is 0
    private static double m_armOffset = 130.9;
    

    public ArmIORobot(int leaderArmMotorID, int followerArmMotorID) {
        m_leaderArmMotor = new CANSparkMax(leaderArmMotorID, MotorType.kBrushless);
        // m_followerArmMotor = new CANSparkMax(followerArmMotorID, MotorType.kBrushless);

        m_leaderArmMotor.restoreFactoryDefaults(); 
        m_leaderArmMotor.setIdleMode(IdleMode.kBrake);
        m_leaderArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        m_leaderArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        // m_followerArmMotor.restoreFactoryDefaults(); 
        // m_followerArmMotor.setIdleMode(IdleMode.kBrake);
        // m_followerArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        // m_followerArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        // Initializes Absolute Encoders from motors
        m_leaderArmEncoder = m_leaderArmMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        // Sets position and velocity conversion factors so units are in degrees and degrees/second
        m_leaderArmEncoder.setPositionConversionFactor(360);
        m_leaderArmEncoder.setVelocityConversionFactor(60);

        m_leaderArmEncoder.setInverted(true);
        m_leaderArmMotor.setInverted(true);

        m_leaderArmEncoder.setZeroOffset((m_armOffset+m_armOffsetDegrees)%360);

        // m_followerArmMotor.follow(m_leaderArmMotor);
    }

    /**
     * <h3>updateInputs</h3>
     * 
     * Left blank because it's only used in simulation
     */
    @Override
    public void updateInputs() {}

    /**
     * <h3>getOutputVoltage</h3>
     * 
     * Gets the shoulder motor outputs in volts
     * @return the sholder motor output voltage
     */
    @Override
    public double getCurrentAngleDegrees() {
        return m_leaderArmEncoder.getPosition();
    }

    /**
     * <h3>getVelocityDegreesPerSecond</h3>
     * 
     * Gets the shoulder motor's velocity
     * @return velocity of arm motor
     */
    @Override
    public double getVelocityDegreesPerSecond() {
        return m_leaderArmEncoder.getVelocity();
    }

    /**
     * <h3>setVoltage</h3>
     * 
     * Set the shoulder motor voltage 
     * @param volts desired voltage
     */
    @Override
    public void setVoltage(double volts) {
        m_leaderArmMotor.setVoltage(volts);
    }

    @Override
    public void adjustOffsetDegrees(double offsetDegrees) {
        m_armOffsetDegrees += offsetDegrees;
        Preferences.setDouble(PREFERENCE_NAME, m_armOffsetDegrees);
        m_leaderArmEncoder.setZeroOffset ((m_armOffset + m_armOffsetDegrees) % 360);
    }
}
