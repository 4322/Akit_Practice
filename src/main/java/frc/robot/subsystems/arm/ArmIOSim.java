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
  private static int instanceNum =
      0; // Line needed due to multiple instantiations of this class causing log overlap

  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(
          new MechanismLigament2d("ArmTower", 30, -90, 6, new Color8Bit(Color.kCoral)));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(new MechanismLigament2d("Arm", 30, 0, 6, new Color8Bit(Color.kYellow)));

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
    SmartDashboard.putData("Arm_Sim " + instanceNum, m_mech2d);
    instanceNum++;
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

    // Update mechanism visualization
    m_arm.setAngle(inputs.positionDeg);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
