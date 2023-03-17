// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer; 


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // constants for auto chooser 
  private static final String kDefaultAuto = "Default: Do nothing";
  // copy this line to have more autos
  private static final String kCustomAuto1 = "Score, charge station";
  private static final String kCustomAuto2 = "Score, drive, veer right";
  private static final String kCustomAuto3 = "Score, drive, veer left";
  private static final String kCustomAuto4 = "Score, drive straight";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // motor objects, differential drive object
  private Talon frontLeft, rearLeft, frontRight, rearRight, intake;
  private Spark barrel, turret;
  private DifferentialDrive robotDrive;
  private Servo intakeDoor;
  
  //sensors
  private Encoder barrelEncoder, turretEncoder;
  private ADIS16470_IMU tilt;
  private Timer gameTimer;

  // joysticks
  private final Joystick d_stick = new Joystick(0);
  private final Joystick arm_stick = new Joystick(1);

  private static final int BUTTON_A = 1;
  private static final int BUTTON_B = 2;
  private static final int BUTTON_X = 3;
  private static final int BUTTON_Y = 4;
  private static final int LEFT_BUMPER = 5;
  private static final int RIGHT_BUMPER = 6;
  //private static final int BACK_BUTTON = 7;
  //private static final int START_BUTTON = 8;
  //private static final int LEFT_STICK_BUTTON = 9;
  private static final int RIGHT_STICK_BUTTON = 10;

  // constants for driving, targeting, motor power
  public double throttle, rot; 
  public double barrelPower, turretPower, intakePower, doorPosition, turretDegrees;
  public double distanceToTarget, angleToTargetRadians;
  public Boolean targetValid, turretSafeToRotate, armTucked;
  public double targetX, targetY, targetArea;
  public double robotTiltAngle;

  // constants for limelight- need to be adjusted for this year's game
  public static final double TARGET_HEIGHT_CM = 60;
  public static final double CAMERA_HEIGHT_CM = 93;
  public static final double heightOfTarget = TARGET_HEIGHT_CM- CAMERA_HEIGHT_CM;
  public static final double GOAL_DISTANCE_FROM_TARGET_CM = 50;
  public static final double MAX_GOAL_DISTANCE_FROM_TARGET_CM = GOAL_DISTANCE_FROM_TARGET_CM + 2;
  public static final double MIN_GOAL_DISTANCE_FROM_TARGET_INCHES = GOAL_DISTANCE_FROM_TARGET_CM - 2;
  
  
  public static final int barrelSafeHeight = 75000;
  public static final int barrelTransportHeight = 40000;
  public static final double turretTicksPerDegree = 1.15;
  public static final int topScore = 250000;
  public static final int midCone = 2350000;

  // initialize variables for timed sections of auto programs
  public double timeToScorePreLoad = 3;
  public double timeToDriveToChargeStation = 2;

  NetworkTableInstance inst = NetworkTableInstance.getDefault(); 
  NetworkTable table = inst.getTable("limelight-cyberus");
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Do nothing", kDefaultAuto);
    // repeat this line to have multiple autos
    m_chooser.addOption("Score, charge station", kCustomAuto1);
    m_chooser.addOption("Score, drive, veer right", kCustomAuto2);
    m_chooser.addOption("Score, drive, veer left", kCustomAuto3);
    m_chooser.addOption("Score, drive, straight", kCustomAuto4);

    // print all auto options in the m_chooser list to the dashboard
    SmartDashboard.putData("Auto choices", m_chooser);

    // motor objects and PWM channels
    frontLeft = new Talon(6);
    rearLeft = new Talon(7);
    frontRight = new Talon(8);
    rearRight = new Talon(9);
    barrel = new Spark(4);
    turret = new Spark(5);
    intake = new Talon (1);
    intakeDoor = new Servo(0);

    // invert left side of drive train
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);
    barrel.setInverted(true);

    // group two motors on each side to always behave the same, make differentialdrive object
    MotorControllerGroup leftSideMotors = new MotorControllerGroup(frontLeft, rearLeft);
    MotorControllerGroup rightSideMotors = new MotorControllerGroup(frontRight, rearRight);
    robotDrive = new DifferentialDrive(leftSideMotors, rightSideMotors);

    // Initialize the encoders
    barrelEncoder = new Encoder(1, 2, false, EncodingType.k4X);
    turretEncoder = new Encoder(3, 4, false, EncodingType.k4X);

    // Initialize the Range Finder */
    //rangeFinder = new AnalogInput(0);

    // initialize the gyro
    tilt = new ADIS16470_IMU();
    tilt.setYawAxis(IMUAxis.kX);
    tilt.calibrate();

    CameraServer.startAutomaticCapture();

    Robot.isReal();

    gameTimer = new Timer();


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    //update camera values
      NetworkTableEntry tx = table.getEntry("tx");
      NetworkTableEntry ty = table.getEntry("ty");
      NetworkTableEntry ta = table.getEntry("ta");
      NetworkTableEntry tv = table.getEntry("tv");
      targetX = tx.getDouble(0.0);
      targetY = ty.getDouble(0.0);
      targetArea = ta.getDouble(0.0);
      targetValid = (tv.getDouble(0.0) == 1.0);


    //Rangefinder values
      //Read the raw value from the sensor //
      //double rangeFinderRawValue = rangeFinder.getValue();
      /** Do math on the value in order to calculate distance w rangefinder */
      /*double voltageScaleFactor = 5/RobotController.getVoltage5V();
      double sensorDistanceCM = rangeFinderRawValue * voltageScaleFactor * 0.125;
      double sensorDistanceIN = rangeFinderRawValue * voltageScaleFactor * 0.0492;
      /** Return the sensor distance to the smart dashboard */
      /*SmartDashboard.putNumber("Distance CM", sensorDistanceCM);
      SmartDashboard.putNumber("Distance IN", sensorDistanceIN); */
    // gyro values 
      
    // turret safety boolean- true if barrel is above safe height, false otherwise
    turretSafeToRotate = barrelEncoder.get() > barrelSafeHeight;
    // calculate turret angular position
    turretDegrees = turretEncoder.get()/ turretTicksPerDegree;
   
    // get current robot tilt angle
    robotTiltAngle = tilt.getAngle();
    
    //post values to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", targetX);
    SmartDashboard.putNumber("LimelightY", targetY);
    SmartDashboard.putNumber("LimelightArea", targetArea);
    SmartDashboard.putNumber("barrel Encoder Ticks", barrelEncoder.get());
    SmartDashboard.putNumber("turret Encoder Ticks", turretEncoder.get());
    SmartDashboard.putBoolean("turret safe?", turretSafeToRotate);
    SmartDashboard.putBoolean("Target Present?", targetValid);
    SmartDashboard.putNumber("chassis rotation", rot);
    SmartDashboard.putNumber("chassis throttle", throttle);
    SmartDashboard.putNumber("distanceToTarget", distanceToTarget);    
    SmartDashboard.putNumber("robot tilt angle", robotTiltAngle); 
    SmartDashboard.putNumber("current timer value in seconds", gameTimer.get()); 
    SmartDashboard.updateValues();       

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
   // System.out.println(" selected: " + m_autoSelected);
    table.getEntry("pipeline").setNumber(3); 

    // reset encoders
    barrelEncoder.reset();
    turretEncoder.reset(); 
    tilt.reset();   
    
    //ensure all motors are off
    robotDrive.arcadeDrive(0, 0);      
    barrel.set(0);
    turret.set(0);
    intake.set(0);

    //intake door set to be closed
    intakeDoor.setPosition(0);

    // reset timer and start it
    gameTimer.reset();
    gameTimer.start();



  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // pick correct auto
    switch (m_autoSelected) {
      // score, get on charge station
      case kCustomAuto1:  
        // lift arm and hold it at scoring height for needed time
        if(gameTimer.get() < timeToScorePreLoad){
          autoSetBarrelHeight(topScore, 0.6);
          // 1 second into that time, start shooting the cube out
          if(gameTimer.get() > 2 && gameTimer.get()<=timeToScorePreLoad){
            intake.set(0.5);
          }
        } // drive to charge station. Stop once you get partially on
        else if(gameTimer.get()>=timeToScorePreLoad){
          autoSetBarrelHeight(barrelSafeHeight, 0.3);
          intake.set(0);
          if(tilt.getAngle()<7){
           robotDrive.arcadeDrive(-0.65, 0);    
          }else{
            robotDrive.arcadeDrive(chassisStabalizer(), 0);
          }
        }
        break;
      
        // score, drive out of community, right side
        case kCustomAuto2:
        if(gameTimer.get() < timeToScorePreLoad){
          autoSetBarrelHeight(topScore, 0.6);
          // 1 second into that time, start shooting the cube out
          if(gameTimer.get() > 2 && gameTimer.get()<=timeToScorePreLoad){
            intake.set(0.5);
          }
        } // drive for 3 seconds, drop barrel, turn off intake
        else if(gameTimer.get()>=timeToScorePreLoad && gameTimer.get() <=timeToScorePreLoad + 2){
          intake.set(0);  
          barrel.set(0.3);
          robotDrive.arcadeDrive(-0.65 , 0.25);
        } else if(gameTimer.get()> timeToScorePreLoad + 2 && gameTimer.get()<timeToScorePreLoad + 3){
          robotDrive.arcadeDrive(0, -0.35);
          barrel.set(0.2);
        } else if(gameTimer.get()>timeToScorePreLoad + 3 && gameTimer.get()<timeToScorePreLoad + 5.5){
            robotDrive.arcadeDrive(-0.65, 0);
            barrel.set(0);
        }
        else if(gameTimer.get()>timeToScorePreLoad+6){ //stop for remaining time
          robotDrive.arcadeDrive(0, 0);
          barrel.set(0);
        }
        break;
      
        case kCustomAuto3:
        if(gameTimer.get() < timeToScorePreLoad){
          autoSetBarrelHeight(topScore, 0.6);
          // 1 second into that time, start shooting the cube out
          if(gameTimer.get() > 2 && gameTimer.get()<=timeToScorePreLoad){
            intake.set(0.5);
          }
        } // drive for 3 seconds, drop barrel, turn off intake
        else if(gameTimer.get()>=timeToScorePreLoad && gameTimer.get() <=timeToScorePreLoad + 2){
          intake.set(0);  
          barrel.set(0.35);
          robotDrive.arcadeDrive(-0.65 , -0.15);
        } else if(gameTimer.get()> timeToScorePreLoad + 2 && gameTimer.get()<timeToScorePreLoad + 3){
          robotDrive.arcadeDrive(0, 0.25);
          barrel.set(0.2);
        } else if(gameTimer.get()>timeToScorePreLoad + 3 && gameTimer.get()<timeToScorePreLoad + 5.5){
            robotDrive.arcadeDrive(-0.65, 0);
            barrel.set(0);
        }
        else if(gameTimer.get()>timeToScorePreLoad+6){ //stop for remaining time
          robotDrive.arcadeDrive(0, 0);
          barrel.set(0);
        }
        break;
        case kCustomAuto4:
        if(gameTimer.get() < timeToScorePreLoad){
          autoSetBarrelHeight(topScore, 0.6);
          // 1 second into that time, start shooting the cube out
          if(gameTimer.get() > 2 && gameTimer.get()<=timeToScorePreLoad){
            intake.set(0.5);
          }  
        }else if(gameTimer.get() > timeToScorePreLoad && gameTimer.get()<timeToScorePreLoad + 4.5){
          intake.set(0);
          robotDrive.arcadeDrive(-0.65, 0);
          barrel.set(0.2);
        } else{
          robotDrive.arcadeDrive(0, 0);
          barrel.set(0);
        }

        // case to do nothing
        case kDefaultAuto:
        default:
        break;
    }
    SmartDashboard.updateValues();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // make sure chassis is stopped
    throttle = 0;
    rot = 0;
    barrelPower = 0;
    turretPower = 0;
    SmartDashboard.updateValues();

    gameTimer.reset();
    gameTimer.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // turret section
    // manual control
    if(arm_stick.getRawAxis(0)>0.25){
      turretPower = -0.5;
    }else if(arm_stick.getRawAxis(0)<-.25){
      turretPower = 0.5;
    }
    // automatic position buttons  
    else if(arm_stick.getRawButton(BUTTON_A)){
      turretPower = setTurretPosition(0, 0.5);
    }else if(arm_stick.getRawButton(BUTTON_B)){
      turretPower = setTurretPosition(90, 0.5); 
    }else if(arm_stick.getRawButton(BUTTON_Y)){
      turretPower = setTurretPosition(180, 0.5);
    }else if(arm_stick.getRawButton(BUTTON_X)){
      turretPower = setTurretPosition(270, 0.5);
    }else{
      turretPower = 0;
    }

    // intake control
    if(d_stick.getRawButton(BUTTON_A)){
      // cube out
      intakePower = 0.5;
    }else if(d_stick.getRawButton(BUTTON_Y)){
      //cube in 
      intakePower = -0.5;
    } else if(d_stick.getRawButton(BUTTON_B)){
      //cone out
      intakePower = -0.85;
    }else{
      intakePower = 0;
    }

    // door
    if(d_stick.getRawButton(RIGHT_BUMPER)){
      // close door
      intakeDoor.setPosition(0);
    } else{
      //door stays open by default
      intakeDoor.setPosition(0.42);
    }

    // driving chassis- throttle damped by square root
    if(d_stick.getRawButton(LEFT_BUMPER)){
      throttle= chassisStabalizer();
    }else if(d_stick.getRawAxis(1)>=0){
      throttle = -0.9*(Math.sqrt(d_stick.getRawAxis(1))); 
    } else if(d_stick.getRawAxis(1)<0){
      throttle = 0.9*(Math.sqrt(d_stick.getRawAxis(1)*-1));
    }
    //driving: rotation is damped by square root
    if(d_stick.getRawButton(BUTTON_X)){
      rot = chassisAutoSeekTarget(); 
    } else if (d_stick.getRawAxis(4) >= 0){
      rot = Math.sqrt(d_stick.getRawAxis(4))*0.8;
    } else if( d_stick.getRawAxis(4)<0){
      rot = -Math.sqrt(-d_stick.getRawAxis(4))*0.8;
    }

    // barrel
    if(arm_stick.getRawAxis(5)<-0.15){
      // manual up
      barrelPower = arm_stick.getRawAxis(5)*0.6; 
    } else if (arm_stick.getRawButton(RIGHT_STICK_BUTTON)){
      barrelPower = 0.1;
    }else if(arm_stick.getRawAxis(5)>0.15){
      // manual down
      barrelPower = 0.6;
    } else if(arm_stick.getRawButton(LEFT_BUMPER)){
      // auto to mid height
      barrelPower = setBarrelPosition(midCone, 0.6);
    }else if(arm_stick.getRawButton(RIGHT_BUMPER)){
      // auto to top height  
      barrelPower = setBarrelPosition(topScore, 0.6);
    }else{
      barrelPower = 0;
    }

    

    // set all motors to their appropriate power values every packet
    robotDrive.arcadeDrive(throttle, rot);      
    barrel.set(barrelPower);
    turret.set(turretPower);
    intake.set(intakePower);
    
    SmartDashboard.updateValues();
    
    // reset gyro to make sure it's accurate right before endgame
    if(gameTimer.get() >= 105 && gameTimer.get()< 107){
      tilt.reset();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


  /*// function to bring arm back to transport mode
  public void setArmToTransportMode(){
      // lift to safe rotation height
      autoSetBarrelPosition(position, power);
      // rotate to starting rotation position 0
      autoSetTurretPosition(position, power);
      // drop arm to "inside chassis" height
      autoSetBarrelPosition(position, power);
  }*/

  //swivel chassis automatically to point at nearest target, in teleop
  public double chassisAutoSeekTarget(){
    // make sure using pipeline3
    table.getEntry("pipeline").setNumber(3); 

    // rotate until target is centered 
    if (targetX > 1.5 && targetValid){ // rotate right if target is left
      rot = -0.3;
    }else if(targetX < -1.5 && targetValid){ //rotate left if target is right
      rot = 0.3;
    }else if(!targetValid){ // no target present, so rotate to find it
      rot = 0;
    }
    return rot;
 }

  // drive to a desired distance from a target using a
  // desired power, requires limelight and target to stay at constant heights
  // requires target to be valid 
  public void limelightDriveToDistance(double desiredDistanceAway, double power){        
    // calculate distance to target
    angleToTargetRadians = targetY * Math.PI / 180;
    distanceToTarget = heightOfTarget / Math.tan(angleToTargetRadians);  

    // drive up to target
    if(distanceToTarget >desiredDistanceAway && targetValid){
      //too far away, so drive closer
      throttle = 0.5;
    }else if(distanceToTarget <desiredDistanceAway && targetValid){ 
      //too close, so reverse
      throttle = -0.5;
    }else{
      throttle = 0;  
    }    
  }


  // function to set turret to desired position using encoder only
  // must hold down button- for teleop
  public double setTurretPosition(int position, double power){  
    int error = turretEncoder.get() - position;
    if (error > 1){
      // turret is left where it should be, so move right.
        turretPower = power;
      }else if(error < -1){
        // turret is right of where it should be, so move left
        turretPower = power;
      }else{
        turretPower = 0;
      }
      return turretPower;
  }

  public double setBarrelPosition(int position, double power){  
    int error = barrelEncoder.get() - position;
    if (error > 10000){
      // barrel is above where it should be, so move down.
        barrelPower = 0.2;
      }else if(error < -10000){
        // barrel is below  where it should be, so move up
        barrelPower = Math.min(((double) error/ (double) position) * 0.6, -0.15);
      }else{
        barrelPower = 0;
      }
      return barrelPower;
  }

  // function to put barrel to desired height using encoder, for auto
  // shows usage of while loop, good for auto but not for teleop
  public void autoSetBarrelHeight(int position, double power){
    int error = barrelEncoder.get() - position;
    if (error > 10000){
      // barrel is above where it should be, so move down.
        barrel.set(power);
      }else if(error < -10000){
        // barrel is below  where it should be, so move up
        barrel.set(-power);
      }else{
        barrel.set(0);
      }
  }

  // auto version of set turret position
  public void autoSetTurretPosition(int position, double power){
    double error = (turretEncoder.get()/ turretTicksPerDegree) - position;
    if (error > 1){
      // turret is left where it should be, so move right.
        turret.set(power);
      }else if(error < -1){
        // turret is right of where it should be, so move left
        turret.set(-power);
      }else{
        turret.set(0);
      }
    SmartDashboard.updateValues();
  }  

  public double chassisStabalizer(){
    // leaning back, so need to drive forwards
    if(robotTiltAngle > 2){
      throttle = 0.75;
    }else if(robotTiltAngle < -2){
      // leaning forward, so need to reverse
      throttle = -0.75;
    }else{
      throttle = 0;
    }
    return throttle;
  }
 /* public void AutoScoreCone(double timer){
    setBarrelHeightAuto(barrelSafeHeight, 0.5);  

  } */
}

