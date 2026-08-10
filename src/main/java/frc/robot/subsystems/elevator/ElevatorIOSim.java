package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
  private double simPosMeters = 0.0;
  private double targetPosMeters = 0.0;
  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    simPosMeters += (targetPosMeters - simPosMeters) * 0.05;

    inputs.posMeters = simPosMeters;
    inputs.appliedVoltage = appliedVoltage;
  }

  @Override
  public void setVoltage(double volts) {
    this.appliedVoltage = volts;
  }

  public void setTargetPosition(double meters) {
    this.simPosMeters = meters;
  }
}
