package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  // 0 degrees is straight ahead and positive is up. The position is reported as +/- 180 degrees.
  
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private int instanceNum;

  public Arm(ArmIO io, int instanceNum) {
    this.io = io;
    this.instanceNum = instanceNum;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm " + instanceNum, inputs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }
}
