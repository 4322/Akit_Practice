package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private static final double minDeg = -180.0;
  private static final double maxDeg = 180.0;

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
    Logger.recordOutput("Arm/AngleDeg", inputs.positionDeg);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }

  // Adding a getter method uses the variables, which fixes the warning!
  public static double getMaxAngleDeg() {
    return maxDeg;
  }

  public static double getMinAngleDeg() {
    return minDeg;
  }
}
