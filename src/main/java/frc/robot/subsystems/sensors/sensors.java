package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class sensors {
  public static DigitalInput intakeSwitch;

  public static void initIntakeSwitch() {
    intakeSwitch = new DigitalInput(1);
  }
}
