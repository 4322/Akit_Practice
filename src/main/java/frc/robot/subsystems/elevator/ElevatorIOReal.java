package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {
  private double requestedVoltage = 0;
  private double position = 0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}

  @Override
  public void setVoltage(double voltage) {
    requestedVoltage = voltage;
  }

  @Override
  public double simPos() {
    position += (requestedVoltage * 0.1);
    Logger.recordOutput("Elevator/actualPos", position);
    return position;
  }
}
