package frc.robot.subsystems.elevator;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;

public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    
    private Timer hewoTimer = new Timer();

    private int instanceNum;

    private enum time {
        ZERO_SECONDS,
        TWO_SECONDS,
        SIX_SECONDS,
        TEN_SECONDS,
        FIFTEEN_SECONDS
    }

   
    public Elevator(ElevatorIO io, int instanceNum) {
      this.io = io;
      this.instanceNum = instanceNum;
    }
  
    @Override
    public void periodic() {
      io.updateInputs(inputs);
      Logger.processInputs("Elevator " + instanceNum, inputs);

      switch (time) {
        case ZERO_SECONDS:
            break;
        case TWO_SECONDS:
            break;
        case SIX_SECONDS:
            break;
        case TEN_SECONDS:
            break;
        case FIFTEEN_SECONDS:
            break;
    }
    }
  
    public void setVoltage(double voltage) {
      io.setVoltage(voltage);
    }
  
    public void setHeight(double targetHeight) {
      io.setHeight(targetHeight);
    }
}