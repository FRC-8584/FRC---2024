package frc.robot.utils;

public class tools {
	//vector (x, y) convert to degrees 
	public static double toDegrees(double x, double y) {
		if(x == 0 && y == 0) return 0;

		double angle = 90 - Math.atan2(y, x) * 57.2957805;//... / 3.1415926 * 180.0

    if(angle < 0) angle += 360;
    else if(angle > 360) angle -= 360;

		return angle;
	}

	//degrees convert to vector (x, y) 
	public static double[] toVector(double radius, double angle) {
    if(angle < 0) angle += 360;
    else if(angle > 360) angle -= 360;

		final double rad = angle * 0.0174533;//angle * 3.1415926 / 180.0

		return new double[]{radius * Math.sin(rad), radius * Math.cos(rad)};
	}

	//bounding 
	public static double bounding(double pos, double min, double max) {
		if(pos < min) pos = min;
		else if(pos > max) pos = max;

		return pos;
	}
}