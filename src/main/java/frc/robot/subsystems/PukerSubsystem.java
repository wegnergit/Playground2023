package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PukerSubsystem extends SubsystemBase {

  private final static double PUKER_DEFAULT_OUTTAKE_SPEED = 0.1;

  private TalonFX m_motor;
  private double m_motorSpeed;

  public PukerSubsystem(int id, double motorSpeed) {
    TalonFXConfiguration cfg;
    m_motor = new TalonFX(id, "rio");
    m_motor.setInverted(true);
    m_motorSpeed = motorSpeed;
  }

  // Overload for default speed if not set
  public PukerSubsystem(int id) {
    this(id, PUKER_DEFAULT_OUTTAKE_SPEED);
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public Command newSetSpeedCommand(double speed) {
    return new InstantCommand(() -> setSpeed(speed));
  }

  // Method to return a command to start the motor with the set speed
  public Command newStartMotorCommand() {
    return newSetSpeedCommand(m_motorSpeed);
  }

  public Command newStopMotorCommand() {
    return newSetSpeedCommand(0.0);
  }

  public Command newReverseMotorCommand() {
    return newSetSpeedCommand(-0.02);
  }
}
