package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private double currentPosition = 0;
  private double Position; // The position of the elevator, in meters
  private double voltage = 5;//need accurate voltage for movement position
  public ElevatorIO io;
  // public ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO log) {
    this.io = log;
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
  public ElevatorIOsim inputs = new ElevatorIOsim();
  private ElevatorState currentState = ElevatorState.TWOSECONDSTART;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Setpoint", currentPosition);
    io.setPosition(currentPosition);
    io.setTargetPosition(Position);
    this.currentPosition =
        (this.currentPosition + 0.05 * (this.Position - this.currentPosition));
    switch (currentState) {
        case TWOSECONDSTART:
        if (homingTimer.hasElapsed(2)){
            Position = 0.4;
            currentState = ElevatorState.SIXSECONDS;
        }
        break;
        case SIXSECONDS:
        if (homingTimer.hasElapsed(6)) {
            Position = 1;
            currentState = ElevatorState.TENSECONDS;
        }
        break;
        case TENSECONDS:
        if (homingTimer.hasElapsed(10)) {
            Position = 0.1;
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
