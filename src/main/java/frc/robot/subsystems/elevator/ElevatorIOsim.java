package frc.robot.subsystems.elevator;

public class ElevatorIOsim implements ElevatorIO {
  private double targetPosition;
  private double position;
  private int instanceCount;
  public ElevatorIOsim(int instanceCount){
    this.instanceCount = instanceCount;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.position = this.position;
    inputs.targetPosition = this.targetPosition; // Simulated voltage, can be adjusted based on simulation needs
  }
  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }
  public void setPosition(double position) {
    this.position = position; // Simulated position, can be adjusted based on simulation needs
  }

}
