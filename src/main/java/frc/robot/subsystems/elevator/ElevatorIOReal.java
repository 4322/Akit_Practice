package frc.robot.subsystems.elevator;

public class ElevatorIOReal implements ElevatorIO {
  private double requestedVoltage = 0;
  private double requestedPosition = 0;

  private double voltage = 0;
  private double position = 0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.position = simPos();
    inputs.voltage = simVolts();
  }

  @Override
  public void setVoltage(double voltage) {
    requestedVoltage = voltage;
  }

  @Override
  public void setPosition(double position) {
    requestedPosition = position;
  }

  private double simVolts() {
    if (voltage < requestedVoltage) {
      voltage += (requestedVoltage - voltage) * 0.05;
    } else {
      voltage -= (voltage - requestedVoltage) * 0.05;
    }
    return voltage;
  }

  private double simPos() {
    if (position < requestedPosition) {
      position += (requestedPosition - position) * 0.05;
    } else {
      position -= (position - requestedPosition) * 0.05;
    }
    return position;
  }
}
