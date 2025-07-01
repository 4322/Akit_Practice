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
    ZERO,
    FORTY_FIVE,
    ONE_HUNDRED_SEVENTY_NINE,
    NEGATIVE_ONE_HUNDRED_SEVENTY_NINE,
    NEGATIVE_NINETY,
    NEGATIVE_FORTY_FIVE,
    ONE_HUNDRED_EIGHTY,
    EIGHTY_FIVE
  }

  private ArmPoints armPoints = ArmPoints.ZERO;

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
      case ZERO:
        setPoint = 45;
        armPoints = ArmPoints.FORTY_FIVE;
      break;
      case FORTY_FIVE:
        if (pid.atGoal()) {
          setPoint = 179;
          armPoints = ArmPoints.ONE_HUNDRED_SEVENTY_NINE;
        }
      break;
      case ONE_HUNDRED_SEVENTY_NINE:
        if (pid.atGoal()) {
          setPoint = -179;
          armPoints = ArmPoints.NEGATIVE_ONE_HUNDRED_SEVENTY_NINE;
        }
      break;
      case NEGATIVE_ONE_HUNDRED_SEVENTY_NINE:
        if (pid.atGoal()) {
          setPoint = -90;
          armPoints = ArmPoints.NEGATIVE_NINETY;
        }
      break;
      case NEGATIVE_NINETY:
        if (pid.atGoal()) {
          setPoint = -45;
          armPoints = ArmPoints.NEGATIVE_FORTY_FIVE;
        }
      break;
      case NEGATIVE_FORTY_FIVE:
        if (pid.atGoal()) {
          setPoint = 180;
          armPoints = ArmPoints.ONE_HUNDRED_EIGHTY;
        }
      break;
      case ONE_HUNDRED_EIGHTY:
        if (pid.atGoal()) {
          setPoint = 85;
          armPoints = ArmPoints.EIGHTY_FIVE;
        }
      break;
      case EIGHTY_FIVE:
        if (pid.atGoal() && true) {
          setPoint = 0;
          armPoints = ArmPoints.ZERO;
        }
      
    }

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
    System.out.println("Type: "+type);
    this.type = type;
    if (Objects.equals("arm0", type)) {
      kP.set(1);
      kD.set(0.1);
    } else if (Objects.equals("arm2", type)) {
      kP.set(2.5);
      kD.set(0.1);
    }
    System.out.println(
        "Type: " + type + " | " + "kP: " + kP.get() + " | kI: " + kI.get() + " | kD: " + kD.get());
  }
}
