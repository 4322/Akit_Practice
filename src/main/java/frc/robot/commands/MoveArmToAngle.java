package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class MoveArmToAngle extends Command {
  private final Arm arm;
  private final double targetAngleDegrees;

  public MoveArmToAngle(Arm arm, double targetAngleDegrees) {
    this.arm = arm;
    this.targetAngleDegrees = targetAngleDegrees;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setTargetAngle(targetAngleDegrees);
  }

  @Override
  public boolean isFinished() {
    return arm.isAtSetPoint();
  }
}
