package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double currentPosition = 0.0;
    public double targetPosition = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setCurrentPosition(double currentPosition) {}

  public default void setTargetPosition(double targetPosition) {}
}
