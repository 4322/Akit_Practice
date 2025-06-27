package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private int instanceNum;
  private double targetPosition;
  private double currentPosition = 0;
  private String message = "0.0 | Position: 0";

  private Timer timer = new Timer();
  private Timer halfSecondTimer = new Timer();

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
    io.setStatus(message);
  }

  public void teleopInit() {
    timer.restart();
    halfSecondTimer.restart();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator " + instanceNum, inputs);

    io.setCurrentPosition(currentPosition);
    io.setTargetPosition(targetPosition);

    this.currentPosition =
        this.currentPosition + 0.05 * (this.targetPosition - this.currentPosition);
    this.message = timer.get() + " | Position: " + this.currentPosition; 

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

    if (halfSecondTimer.hasElapsed(0.5)) {
      halfSecondTimer.reset();
      io.setStatus(message);
      System.out.println(message);
    }
  }
}
