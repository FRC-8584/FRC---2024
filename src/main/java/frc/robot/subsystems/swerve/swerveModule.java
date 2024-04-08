package frc.robot.subsystems.swerve;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.PID;
import frc.robot.utils.tools;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class swerveModule {
	public TalonSRX turningMotor;
	public CANSparkMax driveMotor;

	public String name = "";

	private PID pid;

	//motor power

	private volatile double turnPower;
	private volatile double drivePower;

	//turning degrees of turningmotor

	private volatile double turnValue;

	/**********functions**********/

	//init

	public swerveModule(int turningMotorID, int driveMotorID){
		turningMotor = new TalonSRX(turningMotorID);
		driveMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushed);
	}

	//set motor power

	public void setpoint(double speed, double angle){
		calculate(speed, angle);

		turningMotor.set(TalonSRXControlMode.PercentOutput, -turnPower);
		driveMotor.set(-drivePower);
	}

	//get encoder value

	public void getEncValue(){
		turnValue = -Math.round((double)((int)turningMotor.getSelectedSensorPosition() & 0x03ff) * 0.3515625);

		if(turnValue < 0) turnValue += 360;
		if(turnValue >= 360) turnValue -= 360;

		SmartDashboard.putNumber(name, turnValue);
	}

	//set pid value(kp, ki, kd)
	public void setPID(double kp, double ki, double kd){
		pid = new PID(kp, ki, kd);
	}

	//calculate turningmotor & drivemotor power
	public void calculate(double speed, double angle) {
		double error = angle - turnValue;//SP - PV 

		if(error > 180) error -= 360;
		else if(error < -180) error += 360;

		//speed *= tools.bounding(Math.abs(error / -90.0 + 1.5), 0, 1);//error * ((0 - 1)/(135.0 - 45.0) + 1.5)

		turnPower = tools.bounding(pid.calculate(tools.bounding(error / 45.0, -1, 1)), -1, 1);
		drivePower = speed;
	}
}
