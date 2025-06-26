package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private int instanceNum;

  public Elevator(ElevatorIO io, int instanceNum) {
    this.io = io;
    this.instanceNum = instanceNum;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator " + instanceNum, inputs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setHeight(double targetHeight) {
    io.setHeight(targetHeight);
  }
}
