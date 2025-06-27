package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
  private double targetPosition;
  private double currentPosition;

  private int instanceNum;

  public ElevatorIOSim(int instanceNum) {
    this.instanceNum = instanceNum;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.targetPosition = this.targetPosition;
    inputs.currentPosition = this.currentPosition;
    // System.out.println("Current position: " + this.currentPosition);
    // System.out.println("Target position: " + this.targetPosition);
  }

  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }

  public void setCurrentPosition(double currentPosition) {
    this.currentPosition = currentPosition;
  }
}
