package frc.robot.utils;

public class PID {
	private double kp;
	private double ki;
	private double kd;

	private volatile double Integral;
	private volatile double errorArray[];

	private volatile int thisError;
	private volatile int lastError;

	public PID(double _kp, double _ki, double _kd) {
		kp = _kp;
		ki = _ki;
		kd = _kd;

		resetIntergral();
	}

	public double calculate(double error) {
		thisError++;
		if(thisError == 50) thisError = 0;

		if(thisError == 0) lastError = 49;
		else lastError = thisError - 1;

		//P
		final double P = error * kp;

		//I
		Integral += (error - errorArray[thisError]) * ki;
		final double I = Integral;

		//D
		final double D = (error - errorArray[lastError]) * kd;
		errorArray[thisError] = error;

		return P + I + D;
	}

	public void resetIntergral() {
		errorArray = new double[50];
		Integral = 0;

		thisError = 0;
		lastError = 49;
	}
	
}
