package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  // 0 degrees is straight ahead and positive is up. The position is reported as +/- 180 degrees.

  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private int instanceNum;

  private final PIDController pid = new PIDController(0.8, 0.0, 0.05);

  public Arm(ArmIO io, int instanceNum) {
    this.io = io;
    this.instanceNum = instanceNum;

    pid.setTolerance(1.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm " + instanceNum, inputs);

    double outputVoltage = pid.calculate(getPositionDeg());
    setVoltage(outputVoltage);
  }

  public void setTargetAngle(double targetAngleDegrees) {
    pid.reset();
    pid.setSetpoint(targetAngleDegrees);
    Logger.recordOutput("Arms/TargetAngleDegree", targetAngleDegrees);
  }

  public boolean isAtSetPoint() {
    return pid.atSetpoint();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }
}