package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  @AutoLog
  public static class ElevatorInputs {
    public double positionMeters = 0.0;
    public double targetMeters = 0.0;
  }

  private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  public Elevator() {
    inputs.positionMeters = 0.0;
    inputs.targetMeters = 0.0;
  }

  public void setTargetPosition(double meters) {
    inputs.targetMeters = meters;
  }

  @Override
  public void periodic() {
    double error = inputs.targetMeters - inputs.positionMeters;
    inputs.positionMeters += error * 0.05;

    Logger.processInputs("Elevator", inputs);
  }
}
// Robot should work + able to be tested (for me it does)
// Not done with the advice Mr. Kavner gave me yet on this yet