package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmMove extends Command {

  private LoggedNetworkNumber requestedPositionDeg =
      new LoggedNetworkNumber("Arm/RequestedPositionDeg", 0.0);
  private LoggedNetworkNumber kP = new LoggedNetworkNumber("Arm/kP", 0);
  private LoggedNetworkNumber kI = new LoggedNetworkNumber("Arm/kI", 0);
  private LoggedNetworkNumber kD = new LoggedNetworkNumber("Arm/kD", 0);
  private PIDController armController = new PIDController(0.0, 0.0, 0.0);
  private Arm arm;

  public ArmMove(Arm arm) {
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armController.setPID(kP.get(), kI.get(), kD.get());

    double currentPosition = arm.getPositionDeg();
    double requestedPosition = requestedPositionDeg.get();

    // First check if arm can move optimized
    if (currentPosition < 0) {
      if (requestedPosition > 0) {
        requestedPosition -= 360;
      }
    } else if (currentPosition > 0) {
      if (requestedPosition < 0) {
        requestedPosition += 360;
      }
    }

    if (requestedPosition > 270) {
      requestedPosition -= 360;
    } else if (requestedPosition < -270) {
      requestedPosition += 360;
    }

    double output = armController.calculate(currentPosition, requestedPosition);
    arm.setVoltage(output);

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
