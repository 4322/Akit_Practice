package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import java.util.Objects;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmCommands extends Command {
  private LoggedNetworkNumber requestedPositionDeg =
      new LoggedNetworkNumber("Arm/RequestedPositionDeg", 0.0);
  private LoggedNetworkNumber currentPositionDeg =
      new LoggedNetworkNumber("Arm/CurrentPositionDeg", 0.0);

  private LoggedNetworkNumber kP = new LoggedNetworkNumber("Arm/kP", 0);
  private LoggedNetworkNumber kI = new LoggedNetworkNumber("Arm/kI", 0);
  private LoggedNetworkNumber kD = new LoggedNetworkNumber("Arm/kD", 0);

  private double ownKP = 0;
  private double ownKI = 0;
  private double ownKD = 0;

  private double setPoint = 0;
  private double currentPoint;
  private double output;

  private static int instance;
  private int ownInstance;
  private String type;

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

  ProfiledPIDController pid =
      new ProfiledPIDController(
          0,
          0,
          0,
          new TrapezoidProfile.Constraints(180, 90)); // TODO placeholder values for constraints
  private Arm arm;

  public ArmCommands(Arm arm) {
    this.ownInstance = instance;
    instance++;

    this.arm = arm;
    addRequirements(arm);
    pid.enableContinuousInput(-180.0, 180.0);
    pid.setTolerance(1);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch (armPoints) {
      case FORTY_FIVE:
        setPoint = 45.0;
        armPoints = ArmPoints.ONE_HUNDRED_THIRTY_FIVE;
        break;
      case ONE_HUNDRED_THIRTY_FIVE:
        if (pid.atGoal()) {
          setPoint = 135.0;
          armPoints = ArmPoints.ZERO;
        }
        break;
      case ZERO:
        if (pid.atGoal()) {
          setPoint = 0.0;
          armPoints = ArmPoints.NEGATIVE_ONE_HUNDRED_SEVENTY_NINE;
        }
        break;
      case NEGATIVE_ONE_HUNDRED_SEVENTY_NINE:
        if (pid.atGoal()) {
          setPoint = -179.0;
          armPoints = ArmPoints.ONE_HUNDRED_SEVENTY_NINE;
        }
        break;
      case ONE_HUNDRED_SEVENTY_NINE:
        if (pid.atGoal()) {
          setPoint = 179.0;
          armPoints = ArmPoints.NEGATIVE_NINETY;
        }
        break;
      case NEGATIVE_NINETY:
        if (pid.atGoal()) {
          setPoint = -90.0;
          armPoints = ArmPoints.NINETY;
        }
        break;
      case NINETY:
        if (pid.atGoal()) {
          setPoint = 90.0;
          // Remove to disable loop
          // armPoints = ArmPoints.FORTY_FIVE;
        }
        break;
    }
    requestedPositionDeg.set(setPoint);
    currentPositionDeg.set(arm.getPositionDeg());

    pid.setPID(kP.get(), kI.get(), kD.get());
    // pid.setPID(ownKP, ownKI, ownKD);
    currentPoint = arm.getPositionDeg();
    output = pid.calculate(currentPoint, setPoint);
    arm.setVoltage(output);

    if (Objects.equals("arm1", this.type)) {
      System.out.println("Current: " + currentPoint + " | Target: " + setPoint);
      System.out.println("kP: " + kP.get() + " | kI: " + kI.get() + " | kD: " + kD.get());
      System.out.println(output);
    } else {
      System.out.println(this.type);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}

  public void setType(String type) {
    System.out.println("Type: " + type);
    this.type = type;
    if (Objects.equals("arm0", type)) {
      kP.set(1);
      kD.set(0.1);
    } else if (Objects.equals("arm1", type)) {
      System.out.println("Arm 1 values set");
      kP.set(2.5);
      kD.set(0.1);
      ownKP = 1;
      ownKD = 0.1;
    }
    System.out.println(
        "Type: " + type + " | " + "kP: " + kP.get() + " | kI: " + kI.get() + " | kD: " + kD.get());
  }
}
