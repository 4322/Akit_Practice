package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
public class ArmCommands extends Command {
    private PIDController pidController = new PIDController(0, 0, 0);
    private static final LoggedNetworkNumber kP = new LoggedNetworkNumber("Arm/kP", 0.1);
    private static final LoggedNetworkNumber kI = new LoggedNetworkNumber("Arm/kI", 0.3);
    private static final LoggedNetworkNumber kD = new LoggedNetworkNumber("Arm/kD", 0.7);
    private ArmPosition targetPosition = ArmPosition.ZERO;
     public static enum ArmPosition {
        ZERO,
        FORTYFIVE,
        ONEHUNDREDTHIRTYFIVE,
        ZERO2,
        NEGONEHUNDREDSEVENTYNINE,
        ONEHUNDREDSEVENTYNINE,
        NEGNINETY,
        NINETY;
    }
    @Override
    public void initialize(){
        pidController.setPID(kP.get(), kI.get(), kD.get());
        pidController.enableContinuousInput(-180, 180);
    }
    @Override
    public void execute(){
    switch(targetPosition){
        case ZERO:
        pidController.setSetpoint(0);
        if (pidController.atSetpoint()) {
            // Code to move arm to zero position
            break;
        }
    case FORTYFIVE:
        pidController.setSetpoint(45);
        if (pidController.atSetpoint()) {
            // Code to move arm to 45 degrees
            break;
        }
    case ONEHUNDREDTHIRTYFIVE:
        pidController.setSetpoint(135);
        if (pidController.atSetpoint()) {
            // Code to move arm to 135 degrees
            break;
        }
    case ZERO2:
        pidController.setSetpoint(0);
        if (pidController.atSetpoint()) {
            // Code to move arm to zero position
            break;
        }
    case NEGONEHUNDREDSEVENTYNINE:
        pidController.setSetpoint(-129);
        if (pidController.atSetpoint()) {
            // Code to move arm to -129 degrees
            break;
        }
    case ONEHUNDREDSEVENTYNINE:
        pidController.setSetpoint(129);
        if (pidController.atSetpoint()) {
            // Code to move arm to 129 degrees
            break;
        }
    case NEGNINETY:
        pidController.setSetpoint(-90);
        if (pidController.atSetpoint()) {
            // Code to move arm to -90 degrees
            break;
        }
    case NINETY:
        pidController.setSetpoint(90);
    }
}
}
