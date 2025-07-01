package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmCommands extends Command {
  private LoggedNetworkNumber requestedPositionDeg =
      new LoggedNetworkNumber("Arm/RequestedPositionDeg", 0.0);
  private LoggedNetworkNumber kP = new LoggedNetworkNumber("Arm/kP", 1);
  private LoggedNetworkNumber kI = new LoggedNetworkNumber("Arm/kI", 0);
  private LoggedNetworkNumber kD = new LoggedNetworkNumber("Arm/kD", 0.1);

  private double setPoint = 0;
  private double currentPoint;
  private double output;

  private static int instance;
  private int ownInstance;

  private boolean isInited = false;

  private enum ArmPoints {
    FORTY_FIVE,
    ONE_HUNDRED_THIRTY_FIVE,
    ZERO,
    NEGATIVE_ONE_HUNDRED_SEVENTY_NINE,
    ONE_HUNDRED_SEVENTY_NINE,
    NEGATIVE_NINETY,
    NINETY
  }

  private ArmPoints armPoints = ArmPoints.FORTY_FIVE;

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
    switch (armPoints) {
      case FORTY_FIVE:
        setPoint = 45.0;
        armPoints = ArmPoints.ONE_HUNDRED_THIRTY_FIVE;
        break;
      case ONE_HUNDRED_THIRTY_FIVE:
        if (pid.atSetpoint()) {
          setPoint = 135.0;
          armPoints = ArmPoints.ZERO;
        }
        break;
      case ZERO:
        if (pid.atSetpoint()) {
          setPoint = 0.0;
          armPoints = ArmPoints.NEGATIVE_ONE_HUNDRED_SEVENTY_NINE;
        }
        break;
      case NEGATIVE_ONE_HUNDRED_SEVENTY_NINE:
        if (pid.atSetpoint()) {
          setPoint = -179.0;
          armPoints = ArmPoints.ONE_HUNDRED_SEVENTY_NINE;
        }
        break;
      case ONE_HUNDRED_SEVENTY_NINE:
        if (pid.atSetpoint()) {
          setPoint = 179.0;
          armPoints = ArmPoints.NEGATIVE_NINETY;
        }
        break;
      case NEGATIVE_NINETY:
        if (pid.atSetpoint()) {
          setPoint = -90.0;
          armPoints = ArmPoints.NINETY;
        }
        break;
      case NINETY:
        if (pid.atSetpoint()) {
          setPoint = 90.0;
          // Remove to disable loop
          //armPoints = ArmPoints.FORTY_FIVE;
        }
        break;
    }

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
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
