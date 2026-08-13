package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import org.littletonrobotics.junction.Logger;

public class MoveArmToAngle extends Command {
  private final Arm arm;
  private final double targetAngleDegrees;

  private final PIDController pid = new PIDController(0.5, 0.0, 0.0);

  public MoveArmToAngle(Arm arm, double targetAngleDegrees) {
    this.arm = arm;
    this.targetAngleDegrees = targetAngleDegrees;

    pid.setTolerance(1.0);

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    pid.reset();
    pid.setSetpoint(targetAngleDegrees);
  }

  @Override
  public void execute() {
    double outputVoltage = pid.calculate(arm.getPositionDeg());
    arm.setVoltage(outputVoltage);

    Logger.recordOutput("Arm/TargetAngleDegree", targetAngleDegrees);
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    arm.setVoltage(0);
  }
}
