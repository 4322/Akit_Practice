package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim sim;
  private double appliedVolts = 0.0;
  private int instanceNum = 0;

  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(2, 2);
  private final LoggedMechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 1, 1);
  private final LoggedMechanismLigament2d armTower =
      armPivot.append(
          new LoggedMechanismLigament2d("ArmTower", 1, -90, 6, new Color8Bit(Color.kCoral)));
  private final LoggedMechanismLigament2d arm =
      armPivot.append(new LoggedMechanismLigament2d("Arm", 1, 0, 6, new Color8Bit(Color.kYellow)));

  public ArmIOSim(
      double armMeters,
      double armKg,
      double gearReduction,
      double minDeg,
      double maxDeg,
      int instanceNum) {
    sim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            gearReduction,
            SingleJointedArmSim.estimateMOI(armMeters, armKg),
            armMeters,
            Units.degreesToRadians(minDeg - 2),
            Units.degreesToRadians(maxDeg + 2),
            true,
            0);
    this.instanceNum = instanceNum;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update simulation
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    // Update inputs
    inputs.positionDeg = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocityDegPerSec = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;

    if (sim.hasHitLowerLimit() && inputs.appliedVolts < 0) {
      System.out.println("Arm " + instanceNum + " has hit lower limit!");
      inputs.appliedVolts = 0.0;
    } else if (sim.hasHitUpperLimit() && inputs.appliedVolts > 0) {
      System.out.println("Arm " + instanceNum + " has hit upper limit!");
      inputs.appliedVolts = 0.0;
    }

    // Update mechanism visualization
    arm.setAngle(inputs.positionDeg);
    Logger.recordOutput("Arm_Sim " + instanceNum, mech2d);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
