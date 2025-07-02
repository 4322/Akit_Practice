package frc.robot.subsystems.elevator;

public class ElevatorIOsim implements ElevatorIO {
  private double targetPosition;
  private double currentPosition;


  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.position = currentPosition;
    inputs.targetPosition = this.targetPosition; // Simulated voltage, can be adjusted based on simulation needs
  }

}
