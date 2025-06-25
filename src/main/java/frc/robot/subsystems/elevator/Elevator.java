package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double desiredPos = 0;

  private PIDController controller = new PIDController(1, 0, 0);

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    double volts = controller.calculate(io.simPos(), desiredPos);
    setVoltage(volts);
  }

  public void setPosition(double pos) {
    desiredPos = pos;
    Logger.recordOutput("Elevator/DesiredPos", desiredPos);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }
}
