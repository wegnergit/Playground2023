package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PukerSubsystem extends SubsystemBase {

  private final double PUKER_OUTTAKE_SPEED = 1.0;

  private TalonFX m_motor;
  private double m_motorSpeed;

  public PukerSubsystem(int id, double motorSpeed) {
    m_motor = new TalonFX(id, "rio");
    m_motorSpeed = motorSpeed;
  }

  // Overload for default speed if not set
  public PukerSubsystem(int id) {
    m_motor = new TalonFX(id, "rio");
    m_motorSpeed = PUKER_OUTTAKE_SPEED;
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
}
