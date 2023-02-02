package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private static final double kMetersPerPulse = 0.01;
    private static final double kElevatorMinimumLength = 0.5;
  
    private final PWMSparkMax m_elevatorMotor = new PWMSparkMax(0);
    private final PWMSparkMax m_wristMotor = new PWMSparkMax(1);
    private final AnalogPotentiometer m_wristPot = new AnalogPotentiometer(1, 90);
    private final Encoder m_elevatorEncoder = new Encoder(0, 1);
  
    private MechanismLigament2d m_elevator;
    private MechanismLigament2d m_wrist;

// NOTE: Taken from evalatorsimuation
 // Simulation classes help us simulate what's going on, including gravity.
 private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);
 private static final double kElevatorKp = 5.0;
  private static final double kElevatorGearing = 10.0;
  private static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  private static final double kCarriageMass = 4.0; // kg

  private static final double kMinElevatorHeight = Units.inchesToMeters(2);
  private static final double kMaxElevatorHeight = Units.inchesToMeters(50);
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
    private final EncoderSim m_encoderSim = new EncoderSim(m_elevatorEncoder);
    private final PIDController m_controller = new PIDController(kElevatorKp, 0, 0);
    
    public Arm() {
        m_elevatorEncoder.setDistancePerPulse(kMetersPerPulse);

        // the main mechanism object
        Mechanism2d mech = new Mechanism2d(3, 3);
        // the mechanism root node
        MechanismRoot2d root = mech.getRoot("climber", 2, 0);
    
        // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
        // off the root node or another ligament object
        m_elevator = root.append(new MechanismLigament2d("elevator", kElevatorMinimumLength, 90));
        m_wrist =
            m_elevator.append(
                new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
    
        // post the mechanism to the dashboard
        SmartDashboard.putData("Mech2d", mech);
    
    }

    public void controlArm(double evelatorX, double wristX) {
        //m_elevatorMotor.set(evelatorX);
        m_wristMotor.set(wristX);

        // May switch m_evevatorMotor
        if (evelatorX>0.5) {
            // Here, we run PID control like normal, with a constant setpoint of 30in.
            double pidOutput = m_controller.calculate(m_elevatorEncoder.getDistance(), Units.inchesToMeters(30));
            m_elevatorMotor.setVoltage(pidOutput);
          } else {
            // Otherwise, we disable the motor.
            m_elevatorMotor.set(0.0);
          }
    }

    @Override
    public void periodic() {
        // update the dashboard mechanism's state
        m_elevator.setLength(kElevatorMinimumLength + m_elevatorEncoder.getDistance());
        m_wrist.setAngle(m_wristPot.get());
    }

    @Override
    public void simulationPeriodic() {
        //  TODO not sure (did not do anything without)
        // NOTE: Logic from evalatorsimulation example
        PWMSparkMax m_motor = m_elevatorMotor;
        MechanismLigament2d m_elevatorMech2d = m_elevator;
        
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
    
}
