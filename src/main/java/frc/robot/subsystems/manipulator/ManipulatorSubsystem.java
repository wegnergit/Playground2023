package frc.robot.subsystems.manipulator;

// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ManipulatorSubsystem extends SubsystemBase {
        private final ManipulatorIO m_io;

    public static final double ROLLER_INTAKE_SPEED = 0.2; 
    public static final double HOLD_SPEED = 0.1; 

    public static final double LOW_SCORE_SPEED = -0.1; 
    public static final double MEDIUM_SCORE_SPEED = -0.3;
    public static final double HIGH_SCORE_SPEED = -1.0; 

    public static final double MAX_SPEED = -1.0;

    /**<h3>ManipulatorSubsystem</h3>
     * Decides desired output, in volts, for the manipulator.
     * @param io The ArmIO, use IORobot if robot is real, otherwise use IOSim.
     */
    public ManipulatorSubsystem (ManipulatorIO io) {
        m_io = io;
    }

    /**<h3>periodic</h3>
    * Gets the inputs from the IO, and uses the feed forward and the PID controller to calculate the effort, in volts, to set to the io,
    * for the manipulator motor.
     */
    @Override
    public void periodic() {

        if (DriverStation.isEnabled() || !Robot.isReal()){
            
            this.m_io.updateInputs();
        }
    }

    /**<h3>getRollerVoltage</h3>
     * Gets the voltage of the roller
     * @return getRollerVolate
     */
    public double getRollerVoltage() {
        return m_io.getRollerVoltage();
    }

    public double getRollerCurrent(){
        return m_io.getRollerCurrent();
    }

    /**<h3>getRollerSpeed</h3>
     * Sets the roller speed
     * @return setRollerSpeed
     */
    public void setRollerSpeed(double speed) {
        m_io.setRollerSpeed(speed);
        // Logger.getInstance().recordOutput(this.getClass().getSimpleName() + "/ManipulatorSpeed", speed);
    }

    public Command waitUntilCurrentPast(double amps) { 
        Debouncer debouncer = new Debouncer(.1); //Creates a debouncer to confirm amps are greater than value for .1 seconds
        return Commands.waitUntil(() -> debouncer.calculate(this.getRollerCurrent() > amps));
    }

    public void refollowMotors() {
        m_io.refollow();
    }
}