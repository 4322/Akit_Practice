package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
  private double targetPosition;
  private double currentPosition;
  private String message;

  private int instanceNum;

  public ElevatorIOSim(int instanceNum) {
    this.instanceNum = instanceNum;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.targetPosition = this.targetPosition;
    inputs.currentPosition = this.currentPosition;
    inputs.message = this.message;
    // System.out.println("Current position: " + this.currentPosition);
    // System.out.println("Target position: " + this.targetPosition);
  }

  @Override
  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }

  @Override
  public void setCurrentPosition(double currentPosition) {
    this.currentPosition = currentPosition;
  }

  @Override
  public void setStatus(String message) {
    this.message = message;
  }
}
