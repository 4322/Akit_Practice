package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double posMeters = 0.0;
    public double appliedVoltage = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setElevatorHeight(double targetPosMeters) {}

  public default void setTargetPosition(double targetPosMeters) {}
}
