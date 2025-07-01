package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmCommands extends Command {
  private LoggedNetworkNumber requestedPositionDeg =
      new LoggedNetworkNumber("Arm/RequestedPositionDeg", 0.0);
  private LoggedNetworkNumber kP = new LoggedNetworkNumber("Arm/kP", 0.7);
  private LoggedNetworkNumber kI = new LoggedNetworkNumber("Arm/kI", 0);
  private LoggedNetworkNumber kD = new LoggedNetworkNumber("Arm/kD", 0.2);

  private double setPoint = 0;
  private double currentPoint;
  private double output;

  private static int instance;
  private int ownInstance;

  private boolean isInited = false;


  PIDController pid = new PIDController(0, 0, 0);
  private Arm arm;

  public ArmCommands(Arm arm) {
    this.ownInstance = instance;
    instance++;
    System.out.println("Initialized instance " + ownInstance + " of ArmCommands");
    this.arm = arm;
    addRequirements(arm);
    pid.enableContinuousInput(-180.0, 180.0);
    pid.setTolerance(1);
  }

  @Override
  public void initialize() {
    isInited = true;
  }

  @Override
  public void execute() {
    pid.setPID(kP.get(), kI.get(), kD.get());
    currentPoint = arm.getPositionDeg();
    output = pid.calculate(currentPoint, setPoint);
    arm.setVoltage(output);
    System.out.println("Current: " + currentPoint + " | Target: " + setPoint);
    System.out.println("kP: " + kP.get() + " | kI: " + kI.get() + " | kD: " + kD.get());
    System.out.println(output);
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {}

  public boolean isInitedYet() {
    return isInited;
  }
}
