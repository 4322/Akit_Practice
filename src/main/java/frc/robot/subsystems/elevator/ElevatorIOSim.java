package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
  private double currentPositionMeters = 0.0;
  private double targetPositionMeters = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    double error = targetPositionMeters - currentPositionMeters;
    currentPositionMeters += error * 0.05;

    inputs.positionMeters = currentPositionMeters;
    inputs.targetMeters = targetPositionMeters;
  }

  @Override
  public void setTargetPosition(double meters) {
    this.targetPositionMeters = meters;
  }
}
