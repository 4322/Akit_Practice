package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmCommands extends Command {

  private LoggedNetworkNumber requestedPositionDeg =
      new LoggedNetworkNumber("Arm/RequestedPositionDeg", 0.0);
  private LoggedNetworkNumber kP = new LoggedNetworkNumber("Arm/kP", 1.5);
  private LoggedNetworkNumber kI = new LoggedNetworkNumber("Arm/kI", 0);
  private LoggedNetworkNumber kD = new LoggedNetworkNumber("Arm/kD", 0.05);

  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(90000, 90000);
  private ProfiledPIDController armController = new ProfiledPIDController(0, 0, 0, constraints);
  private Arm arm;
  private Timer timer = new Timer();

  public enum ArmState {
    DEG_NONE,
    DEG_45,
    DEG_135,
    DEG_0,
    DEG_NEG_179,
    DEG_179,
    DEG_NEG_90,
    DEG_90,
    DEG_NEG_45,
    DEG_180,
    DEG_85,
  }

  ArmState armState = ArmState.DEG_NONE;

  public ArmCommands(Arm arm) {
    this.arm = arm;
    this.armController = armController;
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armController.setPID(kP.get(), kI.get(), kD.get());
    double currentPosition = arm.getPositionDeg();
    double requestedPosition = requestedPositionDeg.get();
    double output = armController.calculate(currentPosition, requestedPosition);
    arm.setVoltage(output);
    Logger.recordOutput("Arm/currentPosition", currentPosition);
    Logger.recordOutput("Arm/requestedPositionBeforeOptimize", requestedPosition);

    double uwu = Math.abs(requestedPosition - currentPosition);
    if (currentPosition < 0) {
      if (requestedPosition > 0) {
        if (uwu > 180) {
          requestedPosition -= 360;
        }
      }
    } else if (currentPosition > 0) {
      if (requestedPosition < 0) {
        if (uwu > 180) {
          requestedPosition += 360;
        }
      }
    }

    if (requestedPosition > 270) {
      requestedPosition -= 360;
    } else if (requestedPosition < -270) {
      requestedPosition += 360;
    }

    switch (armState) {
      case DEG_NONE:
        armState = ArmState.DEG_45;
        requestedPositionDeg.set(0.0);
        if (currentPosition >= -0.1 || currentPosition <= 0.1) {
          timer.start();
          if (timer.hasElapsed(0.5)) {
            armState = ArmState.DEG_45;
            timer.stop();
            timer.reset();
          }
        } else {
          timer.stop();
          timer.reset();
        }
        break;
      case DEG_45:
        requestedPositionDeg.set(45.0);
        if (currentPosition <= 45.1 || currentPosition >= 44.9) {
          timer.start();
          if (timer.hasElapsed(1.5)) {
            armState = ArmState.DEG_179;
            timer.stop();
            timer.reset();
          }
        } else {
          timer.stop();
          timer.reset();
        }
        break;
      case DEG_135:
        requestedPositionDeg.set(135.0);
        if (currentPosition <= 135.1 || currentPosition >= 134.9) {
          timer.start();
          if (timer.hasElapsed(1.5)) {
            armState = ArmState.DEG_0;
            timer.stop();
            timer.reset();
          }
        } else {
          timer.stop();
          timer.reset();
        }
        break;
      case DEG_0:
        requestedPositionDeg.set(0.0);
        if (requestedPosition <= 0.1 || currentPosition >= -0.1 || currentPosition >= -0.1) {
          timer.start();
          if (timer.hasElapsed(1.5)) {
            armState = ArmState.DEG_NEG_179;
            timer.stop();
            timer.reset();
          }
        } else {
          timer.stop();
          timer.reset();
        }
        break;
      case DEG_NEG_179:
        requestedPositionDeg.set(-179.0);
        if (currentPosition <= -179.1 || currentPosition >= -178.9) {
          timer.start();
          if (timer.hasElapsed(1.5)) {
            armState = ArmState.DEG_NEG_90;
            timer.stop();
            timer.reset();
          }
        } else {
          timer.stop();
          timer.reset();
        }
        break;
      case DEG_179:
        requestedPositionDeg.set(179.0);
        if (currentPosition <= 179.1 || currentPosition >= 178.9) {
          timer.start();
          if (timer.hasElapsed(1.5)) {
            armState = ArmState.DEG_NEG_179;
            timer.stop();
            timer.reset();
          }
        } else {
          timer.stop();
          timer.reset();
        }
        break;
      case DEG_NEG_90:
        requestedPositionDeg.set(-90.0);
        if (currentPosition >= -90.1 || currentPosition <= -89.9) {
          timer.start();
          if (timer.hasElapsed(1.5)) {
            armState = ArmState.DEG_NEG_45;
            timer.stop();
            timer.reset();
          }
        } else {
          timer.stop();
          timer.reset();
        }
        break;
      case DEG_90:
        requestedPositionDeg.set(90.0);
        if (currentPosition <= 90.1 || currentPosition >= 89.9) {}
        break;
      case DEG_NEG_45:
        requestedPositionDeg.set(-45.0);
        if (currentPosition <= -45.1 || currentPosition >= -44.9) {
          timer.start();
          if (timer.hasElapsed(1.5)) {
            armState = ArmState.DEG_180;
            timer.stop();
            timer.reset();
          }
        } else {
          timer.stop();
          timer.reset();
        }
        break;
      case DEG_180:
        requestedPositionDeg.set(180.0);
        if (currentPosition <= -45.1 || currentPosition >= -44.9) {
          timer.start();
          if (timer.hasElapsed(1.5)) {
            armState = ArmState.DEG_85;
            timer.stop();
            timer.reset();
          }
        } else {
          timer.stop();
          timer.reset();
        }
        break;
      case DEG_85:
        requestedPositionDeg.set(85.0);
        if (currentPosition <= 85.1 || currentPosition >= 84.9) {
          timer.start();
          if (timer.hasElapsed(1.5)) {
            timer.stop();
            timer.reset();
          }
        } else {
          timer.stop();
          timer.reset();
        }
        break;
    }
    Logger.recordOutput("Arm/requestedPosition", requestedPosition);
    Logger.recordOutput("Arm/calculatedVolts", output);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
