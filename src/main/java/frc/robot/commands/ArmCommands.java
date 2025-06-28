package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmCommands extends Command {

  private LoggedNetworkNumber requestedPositionDeg =
      new LoggedNetworkNumber("Arm/RequestedPositionDeg", 0.0);
  private LoggedNetworkNumber kP = new LoggedNetworkNumber("Arm/kP", 0.1);
  private LoggedNetworkNumber kI = new LoggedNetworkNumber("Arm/kI", 0);
  private LoggedNetworkNumber kD = new LoggedNetworkNumber("Arm/kD", 0.0);

  private double setpoint = 0.0;
  private double currentAngle;
  private double output;
  private double instances = 1;
  private PIDController armController = new PIDController(0, 0, 0);
  private Arm arm;
  private static int instance;
  private int ownInstance;
  private int intializedTimes = 0;
  private Timer timer = new Timer();

  private boolean isInited = false;

  public enum ArmState {
    DEG_NONE,
    DEG_45,
    DEG_135,
    DEG_0,
    DEG_NEG_179,
    DEG_179,
    DEG_NEG_90,
    DEG_90
  }

  ArmState armState = ArmState.DEG_NONE;

  public ArmCommands(Arm arm) {
    this.arm = arm;

    addRequirements(arm);
    armController.enableContinuousInput(-180.0, 180.0);
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

    switch (armState) {
      case DEG_NONE:
        armState = ArmState.DEG_45;
        break;
      case DEG_45:
        requestedPositionDeg.set(45.0);
        if (currentPosition <= 45.1 || currentPosition >= 44.9) {
          timer.start();
          if (timer.hasElapsed(0.2)) {
            armState = ArmState.DEG_135;
            timer.stop();
            timer.reset();
          }
        }
        break;
      case DEG_135:
        requestedPositionDeg.set(135.0);
        if (currentPosition == 135.0 ) {
          timer.start();
          if (timer.hasElapsed(0.2)) {
            timer.stop();
            timer.reset();
          }
        }
        break;
      case DEG_0:
        requestedPositionDeg.set(0.0);
        if (requestedPosition == 0.0) {
          timer.start();
          if (timer.hasElapsed(0.2)) {
            armState = ArmState.DEG_NEG_179;
            timer.stop();
            timer.reset();
          }
        }
        break;
      case DEG_NEG_179:
        requestedPositionDeg.set(-179.0);
        if (requestedPosition == -179.0) {
          timer.start();
          if (timer.hasElapsed(0.2)) {
            armState = ArmState.DEG_179;
            timer.stop();
            timer.reset();
          }
        }
        break;
      case DEG_179:
        requestedPositionDeg.set(179.0);
        if (requestedPosition == 179.0) {
          timer.start();
          if (timer.hasElapsed(0.2)) {
            armState = ArmState.DEG_NEG_90;
            timer.stop();
            timer.reset();
          }
        }
        break;
      case DEG_NEG_90:
        requestedPositionDeg.set(-90.0);
        if (requestedPosition == -90.0) {
          timer.start();
          if (timer.hasElapsed(0.2)) {
            armState = ArmState.DEG_90;
            timer.stop();
            timer.reset();
          }
        }
        break;
      case DEG_90:
        requestedPositionDeg.set(90.0);
        if (requestedPosition == 90.0) {}
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
