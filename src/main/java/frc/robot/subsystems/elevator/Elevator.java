package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private double position = 0;
  private double targetPosition; // The position of the elevator, in meters
  private ElevatorIO io;
  private int instanceCount;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO log, int instanceCount) {
    this.instanceCount = instanceCount;
    this.io = log;
  }

  public void teleopInit() {
    homingTimer.reset();
    homingTimer.start();
  }

  private Timer homingTimer = new Timer();

  public enum ElevatorState {
    TWOSECONDSTART, // The elevator is homing
    SIXSECONDS, // The elevator is moving to a position
    TENSECONDS,
    FIFTEENSECONDS; // The elevator is idle
  }

  private ElevatorState currentState = ElevatorState.TWOSECONDSTART;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator" + instanceCount, inputs);
    System.out.println(position);
    System.out.println(targetPosition);
    io.setPosition(position);
    io.setTargetPosition(targetPosition);
    this.position = (this.position + 0.05 * (this.targetPosition - this.position));
    switch (currentState) {
      case TWOSECONDSTART:
        if (homingTimer.hasElapsed(2)) {
          targetPosition = 0.4;
          currentState = ElevatorState.SIXSECONDS;
        }
        break;
      case SIXSECONDS:
        if (homingTimer.hasElapsed(6)) {
          targetPosition = 1;
          currentState = ElevatorState.TENSECONDS;
        }
        break;
      case TENSECONDS:
        if (homingTimer.hasElapsed(10)) {
          targetPosition = 0.1;
          currentState = ElevatorState.FIFTEENSECONDS;
        }
        break;
      case FIFTEENSECONDS:
        if (homingTimer.hasElapsed(15)) {
          System.exit(0);
        }
        break;
    }
  }
}
