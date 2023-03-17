// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // constants for auto chooser 
  private static final String kDefaultAuto = "Default";
  // copy this line to have more autos
  private static final String kCustomAuto1 = "My Auto1";
  private static final String kCustomAuto2 = "My Auto2";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // motor objects, differential drive object
  private Talon frontLeft, rearLeft, frontRight, rearRight;
  private Spark barrel;
  private Spark turret;
  private Spark intake;
  private DifferentialDrive robotDrive;
  private Servo intakeDoor;
  
  //sensors
  private Encoder barrelEncoder, turretEncoder;
  private AnalogInput rangeFinder;
  private ADXRS450_Gyro gyro;
  // joysticks
  private final Joystick d_stick = new Joystick(0);
  private final Joystick arm_stick = new Joystick(1);

  private static final int BUTTON_A = 1;
  private static final int BUTTON_B = 2;
  private static final int BUTTON_X = 3;
  private static final int BUTTON_Y = 4;

  // constants for driving, targeting, motor power
  public double throttle, rot; 
  public double barrelPower, turretPower, intakePower, doorPosition;
  public double distanceToTarget, angleToTargetRadians;
  public Boolean targetValid, turretSafeToRotate, armTucked;
  public double targetX, targetY, targetArea;

  // constants for limelight- need to be adjusted for this year's game
  public static final double TARGET_HEIGHT_INCHES = 25;
  public static final double CAMERA_HEIGHT_INCHES = 9.5;
  public static final double heightOfTarget = TARGET_HEIGHT_INCHES- CAMERA_HEIGHT_INCHES;
  public static final double GOAL_DISTANCE_FROM_TARGET_INCHES = 57;
  public static final double MAX_GOAL_DISTANCE_FROM_TARGET_INCHES = GOAL_DISTANCE_FROM_TARGET_INCHES + 2;
  public static final double MIN_GOAL_DISTANCE_FROM_TARGET_INCHES = GOAL_DISTANCE_FROM_TARGET_INCHES - 2;
  public static final int barrelSafeHeight = 0;
  NetworkTableInstance inst = NetworkTableInstance.getDefault(); 
  NetworkTable table = inst.getTable("limelight-cyberus");
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // repeat this line to have multiple autos
    m_chooser.addOption("My Auto", kCustomAuto1);
    m_chooser.addOption("My Auto", kCustomAuto2);

    // print all auto options in the m_chooser list to the dashboard
    SmartDashboard.putData("Auto choices", m_chooser);

    // motor objects and PWM channels
    frontLeft = new Talon(6);
    rearLeft = new Talon(7);
    frontRight = new Talon(8);
    rearRight = new Talon(9);
    barrel = new Spark(3);
    turret = new Spark(4);
    intake = new Talon (1);
    intakeDoor = new Servo(0);

    // invert left side of drive train
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);

    // group two motors on each side to always behave the same, make differentialdrive object
    MotorControllerGroup leftSideMotors = new MotorControllerGroup(frontLeft, rearLeft);
    MotorControllerGroup rightSideMotors = new MotorControllerGroup(frontRight, rearRight);
    robotDrive = new DifferentialDrive(leftSideMotors, rightSideMotors);

    // Initialize the encoders
    barrelEncoder = new Encoder(1, 2, false, EncodingType.k4X);
    turretEncoder = new Encoder(3, 4, false, EncodingType.k4X);

    // Initialize the Range Finder */
    rangeFinder = new AnalogInput(0);

    // initialize the gyro
    gyro = new  ADXRS450_Gyro();
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
      double rangeFinderRawValue = rangeFinder.getValue();
      /** Do math on the value in order to calculate distance w rangefinder */
      double voltageScaleFactor = 5/RobotController.getVoltage5V();
      double sensorDistanceCM = rangeFinderRawValue * voltageScaleFactor * 0.125;
      double sensorDistanceIN = rangeFinderRawValue * voltageScaleFactor * 0.0492;
      /** Return the sensor distance to the smart dashboard */
      SmartDashboard.putNumber("Distance CM", sensorDistanceCM);
      SmartDashboard.putNumber("Distance IN", sensorDistanceIN);

    // gyro values
      
    // turret safety boolean- true if barrel is above safe height, false otherwise
    //turretSafeToRotate = barrelEncoder.get() > barrelSafeHeight;
    turretSafeToRotate = true;

    Robot.isReal();
    
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
    System.out.println("Auto selected: " + m_autoSelected);
    table.getEntry("pipeline").setNumber(3); 

    // reset encoders
    barrelEncoder.reset();
    turretEncoder.reset();     
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // pick correct auto
    switch (m_autoSelected) {
      case kCustomAuto1:
        // Put custom auto code here
        break;
      case kCustomAuto2:
        // Put more custom auto code here  
      break;
      case kDefaultAuto:
      default:
        // Put default auto code here
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
    SmartDashboard.updateValues();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // operator controls
    if(arm_stick.getRawButton(BUTTON_A)){
        turretPower = 0.2;
    } else if(arm_stick.getRawButton(BUTTON_B)){
      turretPower = -0.2;
    } else if(arm_stick.getRawButton(BUTTON_X)){
          setTurretPosition(1000, 0.2);
    } 
    /*}else if(arm_stick.getRawButton(8)){
      intakePower = 0.2;
    }else if(arm_stick.getRawButton(7)){
      intakePower = 0.2;*/
    else if(arm_stick.getRawAxis(1) > 0){
      intakePower = 0.2;
    }else if(arm_stick.getRawAxis(1) < 0){
      intakePower = -0.2;
    }else if(arm_stick.getRawAxis(3) < 0){
      barrelPower = 0.2;
    }else if(arm_stick.getRawAxis(3) > 0){
      intakePower = -0.2;
    }else{
      /*things that happens when no arm joystick buttons are pressed*/
      throttle = d_stick.getRawAxis(1);
      rot = d_stick.getRawAxis(4);
      intakePower = 0;
      barrelPower = arm_stick.getRawAxis(1);
      turretPower = 0;
    }

    // driver controls
    if(d_stick.getRawButton(BUTTON_A)){
      intakeDoor.set(0);
      //automatic chassis movement stuff?
    } else if(arm_stick.getRawButton(BUTTON_B)){
      intakeDoor.set(1);
    } else{ 
      //drive using joysticks unless automatic chassis movement is 
      //happening in one of the above cases
      throttle = d_stick.getRawAxis(1);
      rot = d_stick.getRawAxis(4);      
    }

    // set all motors to their appropriate power to have
    robotDrive.arcadeDrive(throttle, rot);      
    barrel.set(barrelPower);
    turret.set(turretPower);
    intake.set(intakePower);
    
    SmartDashboard.updateValues();
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


  // function to bring arm back to transport mode
  public void setArmToTransportMode(){
      // lift to safe rotation height
      autoSetBarrelPosition(position, power);
      // rotate to starting rotation position 0
      autoSetTurretPosition(position, power);
      // drop arm to "inside chassis" height
      autoSetBarrelPosition(position, power);
  }

  //swivel turret automatically to point at nearest target, in teleop, if 
  //limelight is on turret
  public void chassisAutoSeekTarget(){
    // make sure using pipeline3
    table.getEntry("pipeline").setNumber(3); 

    // rotate until target is centered 
    if (targetX > calculateRotationMargin() && targetValid){ // rotate right if target is left
      rot = -1*calculateRotation();
    }else if(targetX < -1*calculateRotationMargin() && targetValid){ //rotate left if target is right
      rot = calculateRotation(); 
    }else if(!targetValid){ // no target present, so rotate to find it
      rot = 0.6;
    }
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

  // to determine rotation rate for automatic rotation
  public double calculateRotation() {
    double rotation_value = Math.log(Math.abs(targetX)) / Math.log(29);
    return rotation_value / 1.25;
  }

  // to determine rotation margin for automatic rotation
  public double calculateRotationMargin(){
    double rotMargin = -1 * Math.log(distanceToTarget) / Math.log(Math.pow(100, 1/28.8)) + 29;
    return rotMargin;
  }

  // function to set turret to desired position using encoder only
  // must hold down button- for teleop
  public void setTurretPosition(int position, double power){  
    int error = turretEncoder.get() - position;
    if (error > 2){
      // turret is left where it should be, so move right.
        turretPower = -0.5;
      }else if(error < -2){
        // turret is right of where it should be, so move left
        turretPower = 0.5;
      }else{
        turretPower = 0;
      }
  }

  public void setBarrelPosition(int position, double power){  
    int error = barrelEncoder.get() - position;
    if (error > 2){
      // turret is left where it should be, so move right.
        barrelPower = -0.5;
      }else if(error < -2){
        // turret is right of where it should be, so move left
        barrelPower = 0.5;
      }else{
        barrelPower = 0;
      }
  }

  // function to put barrel to desired height using encoder, for auto
  // shows usage of while loop, good for auto but not for teleop
  public void autoSetBarrelHeight(int height, double power){
    // flip sign of power if barrel is already above target height
    if(barrelEncoder.get() > height){
      power = -power;
    }
    while (height != barrelEncoder.get()){
      barrel.set(power);
    }
    barrel.set(0);
    SmartDashboard.updateValues();
  }

  // auto version of set turret position
  public void autoSetTurretPosition(int height, double power){
    // flip sign of power if barrel is already above target height
    if(turretEncoder.get() > height){
      power = -power;
    }
    while (height != barrelEncoder.get()){
      turret.set(power);
    }
    turret.set(0);
    SmartDashboard.updateValues();
  }  
}

