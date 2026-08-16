package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmCommand extends Command {
  private final Arm arm;

  // Keep the same ProfiledPIDController parameters from Exercise 3
  private final ProfiledPIDController controller =
      new ProfiledPIDController(2.0, 0.0, 0.05, new TrapezoidProfile.Constraints(200.0, 300.0));

  // Exercise 3 target sequence
  private final double[] targets = {45.0, 179.0, -179.0, -90.0, -45.0, 85.0};
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
    // NOTE: Do NOT enable continuous input here, as arm2 has a +/- 270 degree physical limit!
    controller.setTolerance(2.0);
    timer.reset();
    waiting = false;
  }

  @Override
  public void execute() {
    if (currentIndex < targets.length) {
      controller.setGoal(targets[currentIndex]);

      double currentPos = arm.getPositionDeg();
      double output = controller.calculate(currentPos);
      arm.setVoltage(output);

      if (controller.atGoal() && !waiting) {
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

  @Override
  public void end(boolean interrupted) {
    arm.setVoltage(0.0);
  }
}
