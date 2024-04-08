package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.subsystems.swerve.swerve;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.noteSystem.noteSystem;
import frc.robot.subsystems.smartControl.*;
import frc.robot.subsystems.sensors.sensors;
import frc.robot.subsystems.shaft.shaft;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {

  private static Joystick player1 = new Joystick(0);
  private static Joystick player2 = new Joystick(1);

  //variables

  private volatile int shooterSpeed;
  private double intakeTimer = 1000;

  @Override
  public void robotInit() {
    sensors.initIntakeSwitch();
    swerve.init();
    
    shooterSpeed = 0;
  }

  @Override
  public void robotPeriodic() {
    swerve.getEncValue();
    shaft.update();
    System.out.println(shaft.getShaftAngle());
    Limelight.update();
    
  }

  @Override
  public void autonomousInit() {
    Auto.start();
  }

  @Override
  public void autonomousPeriodic() {
    Auto.update3();
    noteSystem.setIntake(Auto.noteSystem[0], false);
    noteSystem.setTransport(Auto.noteSystem[1], false);
    noteSystem.setShooter(Auto.noteSystem[2] ? 1 : 0, 0);
    swerve.move(Auto.swerve[0], Auto.swerve[1], Auto.swerve[2]);

    // if(autoTimer <= 0){
    //   swerve.move(0, 0, 0, false);
    //   shaft.setPower(0);
    // }
    // else{
    //   autoTimer--;
    //   swerve.move(0, 1, 0, false);
    //   shaft.setPower(shaft.kShaftSpeed);
    // }
  }

  @Override
  public void teleopInit() {
    swerve.move(0, 0, 0);
  }

  @Override
  public void teleopPeriodic() {
    doublePlayer();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /************************Functions************************/
  
  public void doublePlayer() {

    /****** Basic control ******/

    //Swerve 
    double x, y, turnRobot;

    //Intake 
    boolean intake_F, intake_R;

    //Transport 
    boolean transport_F, transport_R;

    //Shooter 
    double shooter_F, shooter_R;

    //Shaft 
    double turnShaft;

    //Go straight (robot heading) 
    boolean moveForward;

    /****** Smart control ******/

    //Detect note (limelight) 
    boolean toDetectNote;

    /****** Combination button ******/

    //Intake & shoot combination 
    boolean intakeMod, shootMod;

    /******** Calculate ********/

    // ~ Player1 ~  

    x               =  player1.getRawAxis(0);
    y               = -player1.getRawAxis(1);
    turnRobot       =  player1.getRawAxis(4);

    intakeMod       =  player1.getRawButton(1);
    shootMod        =  player1.getRawButton(2);


    if(player1.getRawButtonPressed(1))intakeTimer = 1000;
    if(intakeMod){
      shaft.setPosition(110);

      intake_F = true;
      intake_R = false;

      transport_F = true;
      transport_R = false;

      if(sensors.intakeSwitch.get()) intakeTimer = 10;

      if(intakeTimer > 0) intakeTimer--;
      else
      {
        intake_F = false;
        intake_R = false;

        transport_F = false;
        transport_R = false;
      }
    }
    else{
      intake_F = false;
      intake_R = false;

      transport_F = false;
      transport_R = false;
    }

    if(shootMod){//20ms * (100/5) = 0.4s
      shaft.setPosition(103);

      if(shooterSpeed >= 100 && 101 < shaft.getShaftAngle() && shaft.getShaftAngle() < 105){
        shooterSpeed = 100;

        transport_F = true;
        transport_R = false;
      }
      else{
        shooterSpeed += 5;//shooter speed up
      }
    }
    else{
      if(!intakeMod){
        transport_F = false;
        transport_R = false;
      }
      
      if(shooterSpeed > 0) shooterSpeed -= 10;//20ms * (100/10) = 0.2s
      if(shooterSpeed < 0) shooterSpeed = 0;
    }

    shooter_F = shooterSpeed / 100.0;
    shooter_R = 0;

    // ~ Player2 ~  

    intake_F        =  intake_F | player2.getRawButton(6);
    intake_R        =  intake_R | player2.getRawButton(5);
 
    transport_F     =  transport_F | player2.getRawButton(8);

    transport_R     =  transport_R | player2.getRawButton(7);

    if(shooter_F < player2.getRawAxis(3)) shooter_F = player2.getRawAxis(3);
    if(shooter_R < player2.getRawAxis(2)) shooter_R = player2.getRawAxis(2);

    toDetectNote    =  player2.getRawButton(3);

    moveForward     =  player2.getRawButton(4);

    if(player2.getPOV() == 0) turnShaft = -shaft.kShaftSpeed;
    else if(player2.getPOV() == 180) turnShaft = shaft.kShaftSpeed;
    else turnShaft = 0;

    if(toDetectNote){
      x = 0;
      y = 0;
      turnRobot = detectNote.getTurnForce();
    }

    if(moveForward){
      x = 0;
      y = 0.5;
      turnRobot = 0;
    }

    /***** Check deadband *****/

    if(-0.05 < x && x < 0.05) x = 0;
    if(-0.05 < y && y < 0.05) y = 0;
    if(-0.05 < turnRobot && turnRobot < 0.05) turnRobot = 0;

    if(shooter_F < 0.05) shooter_F = 0;
    if(shooter_R < 0.05) shooter_R = 0;

    /******* Set point *******/

    swerve.move(x, y, turnRobot);
    
    noteSystem.setIntake(intake_F, intake_R);
    noteSystem.setTransport(transport_F, transport_R);
    noteSystem.setShooter(shooter_F, shooter_R);

    if(!(intakeMod || shootMod)) shaft.setPower(turnShaft);

    SmartDashboard.putBoolean("swtich", shaft.isCalibrate());

  }
}

/*









 
`                   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
`                   * O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O *
`                   * O                                                                     O *
`                   * O                                  _,.=*.                             O *
`                   * O                          __,.=*"'      ".                           O *
`                   * O                         \      _  .=**"'`                           O *
`                   * O                          "===*"H  H                                 O *
`                   * O                                H  H                                 O *
`                   * O                               _H  H___...===***"""***q.             O *
`                   * O           __......====****"""''    ___...===****==.___.b            O *
`                   * O           \      ___...===***""H  H                                 O *
`                   * O            `**""'        qxp   H  H   qxp                           O *
`                   * O                          l l   H  H   l l                           O *
`                   * O                          l l   H  H   l l                           O *
`                   * O                    ___...a l   H  H   l `***""'````*q.              O *
`                   * O             .q"""' ___..., l   H  H   l ..=====..__   \             O *
`                   * O              `a*"''      l l   H  H   l l          ``"'             O *
`                   * O                          l l   H  H   l l             .             O *
`                   * O                          l l   H  H   l l            yH             O *
`                   * O                         _i b   H  H   l l           y H             O *
`                   * O                     _-*' _-*   H  H   b  b         y  j             O *
`                   * O                 _-*' _-*'      H  H    b  '*=====*'  y              O *
`                   * O            .d""' _-*'          H  H     b-_________.d               O *
`                   * O             `a*"'              H  H                                 O *
`                   * O                                q  p                                 O *
`                   * O                                'qp'                                 O *
`                   * O                                                                     O *
`                   * O                                                                     O *
`                   * O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O *
`                   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 









*/
