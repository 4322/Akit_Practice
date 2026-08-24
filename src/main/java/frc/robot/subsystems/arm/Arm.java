package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  // 0 degrees is straight ahead and positive is up. The position is reported as +/- 180 degrees.
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private int instanceNum;

  private final ProfiledPIDController pid =
      new ProfiledPIDController(2.5, 0.05, 0.05, new TrapezoidProfile.Constraints(360.0, 720.0));
  // note to self: 360 = 1 rotation a second max, 720 = still -> 360 in half a second

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
    pid.reset(getPositionDeg());
    pid.setGoal(targetAngleDegrees);
    Logger.recordOutput("Arms/TargetAngleDegree", targetAngleDegrees);
  }

  public boolean isAtSetPoint() {
    return pid.atGoal();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }
}
