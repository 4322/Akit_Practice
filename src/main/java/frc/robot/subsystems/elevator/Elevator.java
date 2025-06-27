package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private int instanceNum;
  private double targetPosition;
  private double currentPosition = 1;

  private Timer timer = new Timer();

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
    timer.restart();
    timer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator " + instanceNum, inputs);

    // System.out.println("Current position: " + this.currentPosition);
    // System.out.println("Target position: " + this.targetPosition);
    io.setCurrentPosition(currentPosition);
    io.setTargetPosition(targetPosition);

    switch (currentTime) {
      case ZERO_SECONDS:
        if (timer.hasElapsed(2)) {
          currentPosition = 0.4;
          System.out.println("Elevator moving to position 0.4");
          currentTime = Timeelapsed.TWO_SECONDS;
        }
        break;
      case TWO_SECONDS:
        if (timer.hasElapsed(6)) {
          currentPosition = 1;
          System.out.println("Elevator moving to position 0.4");
          currentTime = Timeelapsed.SIX_SECONDS;
        }
        break;
      case SIX_SECONDS:
        if (timer.hasElapsed(10)) {
          currentPosition = 0.1;
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
