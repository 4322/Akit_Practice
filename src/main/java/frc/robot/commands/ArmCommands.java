package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmCommands extends Command {
      private double kP = 0.1;
  private double kI = 0;
  private double kD = 0;

  private double setpoint = 0.0;
  private double currentAngle;
  private double output;
}
