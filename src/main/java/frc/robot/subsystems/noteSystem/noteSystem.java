package frc.robot.subsystems.noteSystem;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class noteSystem {
  public static CANSparkMax intake    = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless);
  public static Spark transport       = new Spark(0);//PWM port
  public static CANSparkMax shooter1  = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);
  public static CANSparkMax shooter2  = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);

  public static final double kIntakeMaxSpeed = 1;
  public static final double kTransportSpeed = 0.6;

  public static final double kIntakeTime = 1.5;

  public static void setIntake(boolean intake_F, boolean intake_R) {
    if(intake_F && !intake_R){//forward
      intake.set(-kIntakeMaxSpeed);
    }
    else if(!intake_F && intake_R){//reverse
      intake.set(kIntakeMaxSpeed);
    }
    else intake.set(0);//stop
  }

  public static void setTransport(boolean transport_F, boolean transport_R) {
    if(transport_F && !transport_R){//forward
      transport.set(-kTransportSpeed);
    }
    else if(!transport_F && transport_R){//reversee
      transport.set(kTransportSpeed);
    }
    else transport.set(0);//stop
  }

  public static void setShooter(double shooter_F, double shooter_R) {
    if(shooter_F > 0.05 && shooter_R < 0.05){//forward
      shooter1.set(shooter_F);
			shooter2.set(-shooter_F);
    }
    else if(shooter_F < 0.05 && shooter_R > 0.05){//reverse
			shooter1.set(-shooter_R);
			shooter2.set(shooter_R);
    }
    else{//stop
			shooter1.set(0);
			shooter2.set(0);
    }
  }
}
