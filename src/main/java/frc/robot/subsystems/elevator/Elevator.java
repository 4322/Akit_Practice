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

  private Timer timer = new Timer();
  private Timer hewoTimer = new Timer();

  private enum Timeelapsed {
    ZERO_SECONDS,
    TWO_SECONDS,
    SIX_SECONDS,
    TEN_SECONDS,
    FIFTEEN_SECONDS
  }

  private Timeelapsed currentTime = Timeelapsed.ZERO_SECONDS;

  public Elevator(ElevatorIO io, int instanceNum) {
    this.io = io;
    this.instanceNum = instanceNum;
  }

  public void teleopInit() {
    timer.reset();
    timer.start();
    hewoTimer.reset();
    hewoTimer.start();
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
        (this.currentPosition + 0.05 * (this.targetPosition - this.currentPosition));
    double print = timer.get();
    if (hewoTimer.hasElapsed(0.5)) {
      System.out.println("Elevator " + instanceNum + " current position: " + this.currentPosition);
      System.out.println(
          "Elevator " + instanceNum + " current time elapsed: " + print + " seconds");
      hewoTimer.reset();
    }
    switch (currentTime) {
      case ZERO_SECONDS:
        if (timer.hasElapsed(2)) {
          targetPosition = 0.4;
          currentTime = Timeelapsed.TWO_SECONDS;
        }
        break;
      case TWO_SECONDS:
        if (timer.hasElapsed(6)) {
          targetPosition = 1;
          currentTime = Timeelapsed.SIX_SECONDS;
        }
        break;
      case SIX_SECONDS:
        if (timer.hasElapsed(10)) {
          targetPosition = 0.1;
          currentTime = Timeelapsed.TEN_SECONDS;
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
