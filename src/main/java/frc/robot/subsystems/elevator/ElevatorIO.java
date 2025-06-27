package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ElevatorIO extends SubsystemBase {
      @AutoLog
  public static class ElevatorIOInputs {
    public double positionDeg = 0.0;
    public double height = 0.0;
    public double appliedVolts = 0.0;
  }

  public void updateInputs(ElevatorIOInputs inputs) {}

  public void setVoltage(double volts) {}

  public void setHeight(double height) {}
}
