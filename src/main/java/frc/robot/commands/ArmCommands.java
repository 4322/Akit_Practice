package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmCommands extends Command {

  private LoggedNetworkNumber kP = new LoggedNetworkNumber("Arm/kP", 0);
  private LoggedNetworkNumber kI = new LoggedNetworkNumber("Arm/kI", 0);
  private LoggedNetworkNumber kD = new LoggedNetworkNumber("Arm/kD", 0);

  private double ownKP = 0;
  private double ownKI = 0;
  private double ownKD = 0;

  private double setPoint = 0;
  private double realSetPoint = 0;
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
          new TrapezoidProfile.Constraints(360, 180)); // TODO placeholder values for constraints
  private Arm arm;

  public ArmCommands(Arm arm) {
    this.ownInstance = instance;
    instance++;

    this.arm = arm;
    addRequirements(arm);
    // pid.enableContinuousInput(-270.0, 270.0);
    pid.setTolerance(1);
  }

  @Override
  public void initialize() {
    pid.reset(0);
  }

  @Override
  public void execute() {
    switch (armPoints) {
      case ZERO:
        realSetPoint = 45;
        armPoints = ArmPoints.FORTY_FIVE;
        break;
      case FORTY_FIVE:
        if (pid.atGoal()) {
          realSetPoint = 179;
          armPoints = ArmPoints.ONE_HUNDRED_SEVENTY_NINE;
        }
        break;
      case ONE_HUNDRED_SEVENTY_NINE:
        if (pid.atGoal()) {
          realSetPoint = -179;
          armPoints = ArmPoints.NEGATIVE_ONE_HUNDRED_SEVENTY_NINE;
          System.out.println(179);
        }
        break;
      case NEGATIVE_ONE_HUNDRED_SEVENTY_NINE:
        if (pid.atGoal()) {
          realSetPoint = -90;
          armPoints = ArmPoints.NEGATIVE_NINETY;
          System.out.println(-179);
        }
        break;
      case NEGATIVE_NINETY:
        if (pid.atGoal()) {
          realSetPoint = -45;
          armPoints = ArmPoints.NEGATIVE_FORTY_FIVE;
        }
        break;
      case NEGATIVE_FORTY_FIVE:
        if (pid.atGoal()) {
          realSetPoint = 180;
          armPoints = ArmPoints.ONE_HUNDRED_EIGHTY;
        }
        break;
      case ONE_HUNDRED_EIGHTY:
        if (pid.atGoal()) {
          realSetPoint = 85;
          armPoints = ArmPoints.EIGHTY_FIVE;
        }
        break;
      case EIGHTY_FIVE:
        if (pid.atGoal() && true) { // Prevent loop
          realSetPoint = 0;
          armPoints = ArmPoints.ZERO;
        }
    }
    Logger.recordOutput("realSetPoint", realSetPoint);

    pid.setPID(kP.get(), kI.get(), kD.get());
    // pid.setPID(ownKP, ownKI, ownKD);
    currentPoint = arm.getPositionDeg();
    setPoint = calculateShortestPath(currentPoint, realSetPoint);
    pid.setGoal(setPoint);
    output = pid.calculate(currentPoint);
    arm.setVoltage(output);

    Logger.recordOutput("currentPoint", currentPoint);
    Logger.recordOutput("setPoint", setPoint);
    Logger.recordOutput("kP", kP.get());
    Logger.recordOutput("kI", kI.get());
    Logger.recordOutput("kD", kD.get());
    Logger.recordOutput("Output voltage", output);
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
    } else if (Objects.equals("arm2", type)) {
      kP.set(2.5);
      kD.set(0.1);
    }
    System.out.println(
        "Type: " + type + " | " + "kP: " + kP.get() + " | kI: " + kI.get() + " | kD: " + kD.get());
  }

  private double clampAngle(double angle) {
    double returnAngle = angle % 360;
    if (returnAngle > 270) {
      returnAngle -= 360;
    } else if (returnAngle < -270) {
      returnAngle += 360;
    }

    Logger.recordOutput("Prior angle", angle);
    Logger.recordOutput("Clamped angle", returnAngle);
    return returnAngle;
  }

  private double calculateShortestPath(double currentPoint, double setPoint) {
    double methodCurrentPoint = clampAngle(currentPoint);
    double methodSetPoint = clampAngle(setPoint);


    double difference = methodSetPoint - methodCurrentPoint;
    // Go into range of -180, 540
    difference = difference + 180;
    // Go into range of 0,360
    difference = difference % 360;
    // Ensure result is positive
    difference = difference + 360;
    // Confine result to 0,360
    difference = difference % 360;
    // Confine result to -180,180
    difference = difference - 180;

    Logger.recordOutput("Difference", difference);

    double targetPoint = methodCurrentPoint + difference;
    Logger.recordOutput("Optimized Target Point", clampAngle(targetPoint));
    // Logger.recordOutput("After Difference", difference);
    return clampAngle(targetPoint);
  }
}
