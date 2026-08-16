package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmCommand extends Command {
  private final Arm arm;
  private final PIDController controller = new PIDController(2.1, 0.0, 0.06);
  private final double[] targets = {45.0, 135.0, 179.0, 0.0, -179.0, -90.0, 90.0};
  private int currentIndex = 0;

  private final Timer timer = new Timer();
  private boolean waiting = false;

  public ArmCommand(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    currentIndex = 0;
    controller.enableContinuousInput(-180.0, 180.0);
    controller.setTolerance(2.0);
    timer.reset();
    waiting = false;
  }

  @Override
  public void execute() {
    if (currentIndex < targets.length) {
      // Always keep the controller's setpoint synced to the active target
      controller.setSetpoint(targets[currentIndex]);

      double currentPos = arm.getPositionDeg();
      double output = controller.calculate(currentPos);
      arm.setVoltage(output);

      if (controller.atSetpoint() && !waiting) {
        waiting = true;
        timer.restart();
      }

      if (waiting && timer.hasElapsed(1.0)) {
        waiting = false;
        currentIndex++;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return currentIndex >= targets.length;
  }
}
