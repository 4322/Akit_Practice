package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double targetMeters = 0.0;

    public ElevatorIOInputs() {}
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setTargetPosition(double meters) {}
}
