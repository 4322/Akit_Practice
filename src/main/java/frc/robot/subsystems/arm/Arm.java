package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommands;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  // 0 degrees is straight ahead and positive is up. The position is reported as +/- 180 degrees.

  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private ArmCommands armCommands = new ArmCommands(this);

  private int instanceNum;

  public Arm(ArmIO io, int instanceNum) {
    this.instanceNum = instanceNum;
    System.out.println("Inited arm" + instanceNum);
    this.io = io;
    System.out.println("instance number " + this.instanceNum);

    armCommands.setType("arm" + this.instanceNum);
  }

  public void onInit() {
    if (armCommands.isScheduled()) {
      armCommands.cancel();
    }
    armCommands.schedule();
    System.out.println("Instance " + this.instanceNum + " of arm scheduled");
  }

  @Override
  public void periodic() {
    // System.out.println("Arm running");
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
