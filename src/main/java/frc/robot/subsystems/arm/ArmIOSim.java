package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim sim;
  private double appliedVolts = 0.0;

  public ArmIOSim(double armLengthMeters, double armMassKg, double gearReduction) {
    sim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            gearReduction,
            SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKg),
            armLengthMeters,
            Units.degreesToRadians(-180),
            Units.degreesToRadians(180),
            true,
            0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.positionDeg = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocityDegPerSec = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
