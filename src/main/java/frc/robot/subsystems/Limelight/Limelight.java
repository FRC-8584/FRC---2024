package frc.robot.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("8584");
  private static volatile double tx = 0, ty = 0, ta = 0;
  private static volatile int pipeline = 0;

  private static final double limelightHeight = 12.3;//inch
  private static final double tagHeight = 53.88;
  private static final double speakerHeight = 53.88*2 - 20;
  public static double distance = 0;

  // updata
  public static void update() {
    tx = table.getEntry("tx").getDouble(0.0);
    // ty = table.getEntry("ty").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);

    distance = getDistance();
    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("AimAngle", getAimAngle());
    
  }

  // set pipeline
  public static void setPipeline(int _pipeline) {
    table.getEntry("pipeline").setNumber(_pipeline);
    pipeline = _pipeline;
  }

  // get target datas
  public static double getTX() {return tx;}
  public static double getTY() {return ty;}
  public static double getTA() {return ta;}
  
  // get note angle
  public static double getNoteAngle() {
    return pipeline == 0 ? tx : 0;
  }

  public static double getDistance() {
    return (tagHeight - limelightHeight) / Math.tan(ty * 0.0174533);//inch
  }

  public static double getTagAngle(){
    return ty;
  }

  // get shooter angle to shoot speaker
  public static double getAimAngle() {
    return Math.atan((speakerHeight - limelightHeight) / distance) * 57.2957805;
  }
}
