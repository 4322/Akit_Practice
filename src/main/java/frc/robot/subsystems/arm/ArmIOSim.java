package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim sim;
  private double appliedVolts = 0.0;
  private int instanceNum = 0;

  private final Mechanism2d mech2d = new Mechanism2d(2, 2);
  private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 1, 1);
  private final MechanismLigament2d armTower =
      armPivot.append(new MechanismLigament2d("ArmTower", 1, -90, 6, new Color8Bit(Color.kCoral)));
  private final MechanismLigament2d arm =
      armPivot.append(new MechanismLigament2d("Arm", 1, 0, 6, new Color8Bit(Color.kYellow)));

  public ArmIOSim(
      double armLengthMeters,
      double armMassKg,
      double gearReduction,
      double minAngleDeg,
      double maxAngleDeg,
      int instanceNum) {
    sim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            gearReduction,
            SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKg),
            armLengthMeters,
            Units.degreesToRadians(
                minAngleDeg - 2), // add 2 degree of tolerance to allow for PID oscillation
            Units.degreesToRadians(
                maxAngleDeg + 2), // add 2 degree of tolerance to allow for PID oscillation
            true,
            0);
    SmartDashboard.putData("Arm_Sim " + instanceNum, mech2d);
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
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
