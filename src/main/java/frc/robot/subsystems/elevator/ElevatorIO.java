package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double position = 0.0; // Current position of the elevator
    public double voltage = 0.0; // Current velocity of the elevator
  }

  public default void updateInputs(
      ElevatorIOInputs inputs) {} // Update the inputs for the elevator IO

  public default void setVoltage(double voltage) {} // Set the voltage for the elevator motor

  public default void setPosition(double position) {} // Set the position for the elevator motor
}
