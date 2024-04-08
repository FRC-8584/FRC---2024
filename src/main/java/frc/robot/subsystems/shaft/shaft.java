package frc.robot.subsystems.shaft;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class shaft {
  private static CANSparkMax shaft = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless);

  private static DigitalInput limit = new DigitalInput(0);

  private static volatile double kEncMaxLimit = 35;
  private static volatile double angle = 90;
  private static boolean Calibrate = false;
  
  public static final double kShaftSpeed = 0.1;

  public static final double kMinLimit = 50;//intake
  public static final double kMaxLimit = 125;//shooter

  public static void update() {
    if(limit.get()){
      kEncMaxLimit = 2.88 * shaft.getEncoder().getPosition();
      Calibrate = true;
    }  

    //360 = 2.88 * 125, 1 unit = 2.88 degrees
    angle = kMaxLimit - (kEncMaxLimit - 2.88 * shaft.getEncoder().getPosition());
  }

  public static double getShaftAngle() {
    return angle;
  }

  public static boolean isCalibrate() {
    return Calibrate;
  }


  public static void setPosition(double degrees) {
    if(degrees > kMaxLimit) degrees = kMaxLimit;
    else if(degrees < kMinLimit) degrees = kMinLimit;

    
    if(angle < degrees - 20) shaft.set(kShaftSpeed * 4);
    else if(angle > degrees + 20) shaft.set(-kShaftSpeed * 4);
    else if(angle < degrees - 7) shaft.set(kShaftSpeed * 3);
    else if(angle > degrees + 7) shaft.set(-kShaftSpeed * 3);
    else if(angle < degrees - 4) shaft.set(kShaftSpeed * 2);
    else if(angle > degrees + 4) shaft.set(-kShaftSpeed * 2);
    else if(angle < degrees - 1) shaft.set(kShaftSpeed);
    else if(angle > degrees + 1) shaft.set(-kShaftSpeed);

    else shaft.set(0);
  }

  /*
   * Set power in percent output
   */
  public static void setPower(double power) {
    if(limit.get()){
      if(power > 0) power = 0;
    }
    else if(angle <= kMinLimit){
      if(power < 0) power = 0;
    }

    shaft.set(power);
  }

}
