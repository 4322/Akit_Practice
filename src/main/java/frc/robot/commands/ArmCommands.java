package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmCommands extends Command {
  private double kP = 0.6;
  private double kI = 0;
  private double kD = 0;

  private double setPoint = 0;
  private double currentPoint;
  private double output;

  private static int instance;
  private int ownInstance;
  private int intializedTimes = 0;

  private boolean isInited = false;

  double[] queuedPositions = {45, 135, 0, -179, 179, -90, 90};
  int timesRun = 0;

  PIDController pid = new PIDController(kP, kI, kD);

  private Arm arm;

  public ArmCommands(Arm arm) {
    this.ownInstance = instance;
    instance++;
    System.out.println("Initialized instance " + ownInstance + " of ArmCommands");
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    isInited = true;
    System.out.println("Initialized " + intializedTimes + " times");
    intializedTimes++;
    System.out.println("Set point to " + queuedPositions[timesRun]);
    setPoint = queuedPositions[timesRun];
    timesRun++;
  }

  @Override
  public void execute() {
    currentPoint = arm.getPositionDeg();
    output = pid.calculate(currentPoint, setPoint);
    arm.setVoltage(output);
    System.out.println("Current: " + arm.getPositionDeg() + " | Target: " + setPoint);
    // System.out.println(output);
  }

  @Override
  public boolean isFinished() {
    return MathUtil.isNear(setPoint, currentPoint, 1);
  }

  @Override
  public void end(boolean interrupted) {}

  public boolean isInitedYet() {
    return isInited;
  }
}
