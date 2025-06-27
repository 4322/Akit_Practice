package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommands;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  // 0 degrees is straight ahead and positive is up. The position is reported as +/- 180 degrees.

  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private ArmCommands armCommands = new ArmCommands(this);

  private int commandsScheduled = 0;

  private int instanceNum;

  public Arm(ArmIO io, int instanceNum) {
    this.io = io;
    this.instanceNum = instanceNum;
  }

  public void onInit() {
    armCommands.schedule();
    commandsScheduled++;
  }

  @Override
  public void periodic() {
    // System.out.println("Arm running");
    io.updateInputs(inputs);
    Logger.processInputs("Arm " + instanceNum, inputs);

    if (commandsScheduled < 6 && !armCommands.isScheduled()) {
      commandsScheduled++;
      armCommands.schedule();
    }
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }
}
