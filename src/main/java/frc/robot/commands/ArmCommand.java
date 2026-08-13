package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmCommand extends Command {
  private final Arm arm;
  private final PIDController controller = new PIDController(2, 0.0, 0.0);
  private final double[] targets = {45.0, 135.0, 0.0, -179.0, 179.0, -90.0, 90.0};
  private int currentIndex = 0;

  public ArmCommand(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    currentIndex = 0;
    controller.enableContinuousInput(-180.0, 180.0);
    if (targets.length > 0) {
      controller.setSetpoint(targets[currentIndex]);
    }
  }

  @Override
  public void execute() {
    if (currentIndex < targets.length) {
      double currentPos = arm.getPositionDeg();
      double output = controller.calculate(currentPos);
      arm.setVoltage(output);

      if (controller.atSetpoint()) {
        currentIndex++;
        if (currentIndex < targets.length) {
          controller.setSetpoint(targets[currentIndex]);
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return currentIndex >= targets.length;
  }

  @Override
  public void end(boolean interrupted) {
    arm.setVoltage(0.0);
  }
}
