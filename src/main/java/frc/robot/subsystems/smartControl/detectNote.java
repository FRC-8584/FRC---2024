package frc.robot.subsystems.smartControl;

import frc.robot.utils.PID;
import frc.robot.subsystems.Limelight.Limelight;

public class detectNote {
  private static PID pid = new PID(1.5, (1e-6), 2.1);//ki = 10^-6

  public static double getTurnForce() { 
    if(Limelight.getTX()<1 && Limelight.getTX()>-1){
      pid.resetIntergral();
      return 0;
    }
    return pid.calculate((Limelight.getTA() > 0.1 ? Limelight.getTX() : 0 )/32.0);
  }
}
