package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EvalatorSimuationSample extends SubsystemBase {

    private static final int kMotorPort = 15;
    private static final int kEncoderAChannel = 15;
    private static final int kEncoderBChannel = 16;
  
    private static final double kElevatorKp = 5.0;
    private static final double kElevatorGearing = 10.0;
    private static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    private static final double kCarriageMass = 4.0; // kg
  
    private static final double kMinElevatorHeight = Units.inchesToMeters(2);
    private static final double kMaxElevatorHeight = Units.inchesToMeters(50);
  
    // distance per pulse = (distance per revolution) / (pulses per revolution)
    //  = (Pi * D) / ppr
    private static final double kElevatorEncoderDistPerPulse =
        2.0 * Math.PI * kElevatorDrumRadius / 4096;
  
    private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(14);
  
    // Standard classes for controlling our elevator
    private final PIDController m_controller = new PIDController(kElevatorKp, 0, 0);
    private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
    private final PWMSparkMax m_motor = new PWMSparkMax(kMotorPort);
    
    // Simulation classes help us simulate what's going on, including gravity.
    private final ElevatorSim m_elevatorSim =
        new ElevatorSim(
            m_elevatorGearbox,
            kElevatorGearing,
            kCarriageMass,
            kElevatorDrumRadius,
            kMinElevatorHeight,
            kMaxElevatorHeight,
            true,
            VecBuilder.fill(0.01));
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  
    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d(
                "Elevator", Units.metersToInches(m_elevatorSim.getPositionMeters()), 90));
  
    public EvalatorSimuationSample() {
        m_encoder.setDistancePerPulse(kElevatorEncoderDistPerPulse);
  
        // Publish Mechanism2d to SmartDashboard
        // To view the Elevator Sim in the simulator, select Network Tables -> SmartDashboard ->
        // Elevator Sim
        SmartDashboard.putData("Elevator Sim", m_mech2d);
      }
    
  
    @Override
    public void simulationPeriodic() {
      // In this method, we update our simulation of what our elevator is doing
      // First, we set our "inputs" (voltages)
      m_elevatorSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
  
      // Next, we update it. The standard loop time is 20ms.
      m_elevatorSim.update(0.020);
  
      // Finally, we set our simulated encoder's readings and simulated battery voltage
      m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
      // SimBattery estimates loaded battery voltages
      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  
      // Update elevator visualization with simulated position
      m_elevatorMech2d.setLength(Units.metersToInches(m_elevatorSim.getPositionMeters()));
    }
  
    // @Override
    // public void teleopPeriodic() {
    public void controlEvalator(boolean buttonPressed) {
      if (buttonPressed) {
        // Here, we run PID control like normal, with a constant setpoint of 30in.
        double pidOutput = m_controller.calculate(m_encoder.getDistance(), Units.inchesToMeters(30));
        m_motor.setVoltage(pidOutput);
      } else {
        // Otherwise, we disable the motor.
        m_motor.set(0.0);
      }
    }
  
    // @Override
    // public void disabledInit() {
    //   // This just makes sure that our simulation code knows that the motor's off.
    //   m_motor.set(0.0);
    // }
  
}
