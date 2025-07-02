package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private double currentPosition = 0;
  private double setposition; // The position of the elevator, in meters
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

  private ElevatorState currentState = ElevatorState.TWOSECONDSTART;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Setpoint", position);
    switch (currentState) {
        case TWOSECONDSTART:
        if (homingTimer.hasElapsed(2)){
            io.setVoltage(voltage);
            position = 0.4;
            io.setPosition(position);
            currentState = ElevatorState.SIXSECONDS;
        }
        break;
        case SIXSECONDS:
        if (homingTimer.hasElapsed(6)) {
            io.setVoltage(voltage);
            position = 1;
            io.setPosition(position);
            currentState = ElevatorState.TENSECONDS;
        }
        break;
        case TENSECONDS:
        if (homingTimer.hasElapsed(10)) {
            io.setVoltage(voltage);
            position = 0.1;
            io.setPosition(position);
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
