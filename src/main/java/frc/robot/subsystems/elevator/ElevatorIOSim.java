package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim sim;
  private double targetHeight = 0.0;
  private double appliedVolts = 0.0;

  private final Mechanism2d mech2d = new Mechanism2d(2, 2);
  private final MechanismRoot2d elevatorBottom = mech2d.getRoot("ElevatorBottom", 1, 0);

  /*
  public ElevatorIOSim(
      double elevatorHeightMeters,
      double elevatorCarriageMassKg,
      double elevatorMaxHeightMeters,
      double elevatorStartingHeightMeters
  ) {
      sim =
      new ElevatorSim(null, null, elevatorHeightMeters, elevatorMaxHeightMeters, true, elevatorStartingHeightMeters, null)
  }
  */

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.setInputVoltage(targetHeight);
    sim.update(0.02);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setHeight(double height) {
    targetHeight = MathUtil.clamp(height, 0, 20);
  }
}
