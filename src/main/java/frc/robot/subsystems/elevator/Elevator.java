package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private int cycleCounter = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    cycleCounter++;
    if (cycleCounter >= 25) {
      double timestampSec = Logger.getTimestamp() / 1e6;
      //1e6 converts microsecs to secs - google
      System.out.println("Time " + timestampSec + "s, Elevator Pos: " + inputs.positionMeters + "m");

      cycleCounter = 0;
      }
    }

  public void setTargetPosition(double meters) {
    io.setTargetPosition(meters);
  }
}
