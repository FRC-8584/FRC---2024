package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.PID;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.shaft.shaft;
import frc.robot.subsystems.sensors.sensors;

public class Auto {
  // Variables
  public static double[] swerve = new double[3];         // drift, forward, turn
  public static boolean[] noteSystem = new boolean[3];   // intake, transport, shooter
  public static boolean[] last_noteSystem = new boolean[3];
  public static double aimShaftAngle = 0;                // shaft aim angle
  public static boolean isStored = false, isTarget = false;
  
  // Sensors
  private static double[] target = new double[3];         // tx, ty, ta

  private static int ShooterTimer = 50;

  // Status
  private static double startTime = 0;
  private static boolean readyToGetNote = false;

  // Timer
  private static int intakeTimer = 70;

  // Tools
  private static PID pid = new PID(1.1, 1e-6, 2.2);

  /********** functions **********/


  // start auto
  public static void start(){
    startTime = Timer.getFPGATimestamp();

    swerve[0] = 0;
    swerve[1] = 0;
    swerve[2] = 0;

    noteSystem[0] = false;
    noteSystem[1] = false;
    noteSystem[2] = false;
  }

  public static void update4(){

    double time = Timer.getFPGATimestamp() - startTime;

    last_noteSystem[0] = noteSystem[0];
    last_noteSystem[1] = noteSystem[1];
    last_noteSystem[2] = noteSystem[2];

    if(time > 0 && time < 2.5)
    {
      if(!shaft.isCalibrate()) shaft.setPower(shaft.kShaftSpeed);
    }
    else if(time > 2.5 && time < 3.5){
      shaft.setPosition(103);
    }
    else if(time > 3.5 && time < 5){
      shaft.setPosition(103);
      shoot();
    }
    else if(time > 5 && time < 7)
    {
      noteSystem[0] = false;
      noteSystem[1] = false;
      noteSystem[2] = false;

      swerve[0] = 0;
      swerve[1] = 0;
      swerve[2] = 0;

      towardNote();
    }
    else if(!isStored)
    {
      if(time > 7 && time < 11)
      {
        swerve[0] = 0;
        swerve[1] = -0.5;
        swerve[2] = 0;
      }
      else if(time > 11 && time < 12.5)
      {
        shoot();
      }
      else if(time > 12.5 && time < 13.5)
      {
        swerve[0] = 0;
        swerve[1] = 1;
        swerve[2] = 0;

        noteSystem[0] = false;
        noteSystem[1] = false;
        noteSystem[2] = false;
      }
    }
    else{
      swerve[0] = 0;
      swerve[1] = 0;
      swerve[2] = 0;

      noteSystem[0] = false;
      noteSystem[1] = false;
      noteSystem[2] = false;
    }
  }

  public static void update3(){

    double time = Timer.getFPGATimestamp() - startTime;

    last_noteSystem[0] = noteSystem[0];
    last_noteSystem[1] = noteSystem[1];
    last_noteSystem[2] = noteSystem[2];

    if(time > 0 && time < 2.5)
    {
      if(!shaft.isCalibrate()) shaft.setPower(shaft.kShaftSpeed);
    }
    else if(time > 2.5 && time < 3.5){
      shaft.setPosition(103);
    }
    else if(time > 3.5 && time < 5){
      shaft.setPosition(103);
      shoot();
    }
    else if(time > 5 && time < 6)
    {
      swerve[0] = 0;
      swerve[1] = 0.8;
      swerve[2] = 0;

      noteSystem[0] = false;
      noteSystem[1] = false;
      noteSystem[2] = false;
    }
    else
    {
      swerve[0] = 0;
      swerve[1] = 0;
      swerve[2] = 0;

      noteSystem[0] = false;
      noteSystem[1] = false;
      noteSystem[2] = false;
    }
  }

  public static void update2(){
    double time = Timer.getFPGATimestamp() - startTime;

    last_noteSystem[0] = noteSystem[0];
    last_noteSystem[1] = noteSystem[1];
    last_noteSystem[2] = noteSystem[2];

    if(time > 0 && time < 2)
    {
      if(!shaft.isCalibrate())
      {
        shaft.setPower(shaft.kShaftSpeed);
      }

      swerve[0] = 0;
      swerve[1] = -0.5;
      swerve[2] = 0;
    }
    else if(time > 2 && time < 4)
    {
      swerve[0] = 0;
      swerve[1] = 0;
      swerve[2] = 0;
      shoot();
    }
    else if(time > 4 && time < 5)
    {
      swerve[0] = 0;
      swerve[1] = 0.7;
      swerve[2] = 0;

      noteSystem[0] = false;
      noteSystem[1] = false;
      noteSystem[2] = false;
    }
    else if(time > 5)
    {
      lookingForNote();
    }

  }

  public static void towardNote(){
    double time = Timer.getFPGATimestamp() - startTime;

    if(time > 7) return;

    if(!isStored)
    {
      swerve[0] = 0;
      swerve[1] = 1;
      swerve[2] = 0;
      
      noteSystem[0] = true;
      noteSystem[1] = true;
      noteSystem[2] = false;
    }
    
    if((!last_noteSystem[0] && noteSystem[0]) && (!last_noteSystem[1] && noteSystem[1])) intakeTimer = 1500;
    
    if(sensors.intakeSwitch.get()) intakeTimer = 10;
    else
    {
      if(intakeTimer > 0) intakeTimer--;
      else if(intakeTimer == 0)
      {
        noteSystem[0] = false;
        noteSystem[1] = false;

        swerve[0] = 0;
        swerve[1] = 0;
        swerve[2] = 0;

        isStored = true;
        intakeTimer--;
      }
    }

  }

  private static void lookingForNote(){
    // change limelight pipeline to detect note
    Limelight.setPipeline(0);

    if(!readyToGetNote){
      // detected note
      if(target[2] > 0.2){
        // set values
        swerve[0] = 0;
        swerve[1] = yCalculator(target[0]);
        swerve[2] = turnCalculator(target[0]);
      }
      
      // ready to get note
      if(target[2] > 2 && (target[0] < 1 && target[0] > -1)){
        readyToGetNote = true;
      }

      // didn't detect note
      else{
        swerve[0] = 0;
        swerve[1] = 0.8;
        swerve[2] = 1;            
      }
    }
    else{
      // set values
      swerve[0] = 0;
      swerve[1] = 0.8;
      swerve[2] = turnCalculator(target[0]);
      noteSystem[0] = true;
      noteSystem[1] = true;
      noteSystem[2] = false;
    }
    
    
    if((!last_noteSystem[0] && noteSystem[0]) && (!last_noteSystem[1] && noteSystem[1])) intakeTimer = 1500;
    // is got note ?
    if(sensors.intakeSwitch.get()){
      intakeTimer = 10;
    }
    else
    {
      if(intakeTimer > 0) intakeTimer--;
      else if(intakeTimer == 0)
      {
        noteSystem[0] = false;
        noteSystem[1] = false;
        intakeTimer--;
        swerve[0] = 0;
        swerve[1] = 0;
        swerve[2] = 0;
      }
    }

  }

  private static void shoot(){
    noteSystem[2] = true;
    if(ShooterTimer > 0) ShooterTimer--;
    
    if(ShooterTimer < 40) noteSystem[1] = true;

  }

  // h
  private static double yCalculator(double tx){
    if(tx == 0) return 1;
    else return 1/tx;
  }

  //
  private static double turnCalculator(double err){
    double turn = 0;

    if(err == 0) turn = 0;
    else turn = pid.calculate(err);

    if(err < 1 && err > -1){
      turn = 0;
      pid.resetIntergral();
    }
    
    return turn;
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