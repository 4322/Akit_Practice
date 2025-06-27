package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private int instanceNum;
  private double targetPosition;
  private double currentPosition = 0;

  private Timer timer = new Timer();

  private timePast time = timePast.ZERO_SECONDS;

  private enum timePast {
    ZERO_SECONDS,
    TWO_SECONDS,
    SIX_SECONDS,
    TEN_SECONDS,
    FIFTEEN_SECONDS
  }

  public Elevator(ElevatorIO io, int instanceNum) {
    this.io = io;
    this.instanceNum = instanceNum;
  }

  public void teleopInit() {
    timer.restart();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator " + instanceNum, inputs);

    // System.out.println("Current position: " + this.currentPosition);
    // System.out.println("Target position: " + this.targetPosition);
    io.setCurrentPosition(currentPosition);
    io.setTargetPosition(targetPosition);

    this.currentPosition =
        this.currentPosition + 0.05 * (this.targetPosition - this.currentPosition);

    switch (time) {
      case ZERO_SECONDS:
        if (timer.hasElapsed(2)) {
          this.targetPosition = 0.4;
          time = timePast.TWO_SECONDS;
        }
        break;
      case TWO_SECONDS:
        if (timer.hasElapsed(6)) {
          this.targetPosition = 1;
          time = timePast.SIX_SECONDS;
        }
        break;
      case SIX_SECONDS:
        if (timer.hasElapsed(10)) {
          this.targetPosition = 0.1;
          time = timePast.TEN_SECONDS;
        }
        break;
      case TEN_SECONDS:
        if (timer.hasElapsed(15)) {
          System.exit(0);
        }
        break;
      case FIFTEEN_SECONDS:
        break;
    }
  }

}
