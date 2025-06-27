package frc.robot.subsystems.elevator;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    time currentTime = time.ZERO_SECONDS;
   
    public Elevator(ElevatorIO io, int instanceNum) {
      this.io = io;
      this.instanceNum = instanceNum;
    }
  
    @Override
    public void periodic() {
      io.updateInputs(inputs);
      Logger.processInputs("Elevator " + instanceNum, inputs);

    // Example: get current time state (replace with actual logic as needed) // TODO: Replace with real logic

    switch (currentTime) {
      case ZERO_SECONDS: 
    if (hewoTimer.hasElapsed(2)) {
        currentTime = time.TWO_SECONDS;
    }
      break;
      case TWO_SECONDS:
        if (hewoTimer.hasElapsed(6)) {
            currentTime = time.SIX_SECONDS;}

       break;
      case SIX_SECONDS:
            if (hewoTimer.hasElapsed(10)) {
                currentTime = time.TEN_SECONDS;
            }
      break;
      case TEN_SECONDS:
            if (hewoTimer.hasElapsed(15)) {
                currentTime = time.FIFTEEN_SECONDS;
            }
      break;
      case FIFTEEN_SECONDS:
            if (hewoTimer.hasElapsed(20)) {
                currentTime = time.ZERO_SECONDS;
                hewoTimer.reset();
            }
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