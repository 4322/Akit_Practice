package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private double targetPosMeters = 0.0;

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
    Logger.recordOutput("Elevator " + instanceNum + "/TargetPosMeters", targetPosMeters);
  }

  public void setElevatorHeight(double targetPosMeters) {
    this.targetPosMeters = targetPosMeters;
    io.setTargetPosition(targetPosMeters);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double getPositionMeters() {
    return inputs.posMeters;
  }
}
