/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.networktables.*;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // BEGIN Declare and Attach CAN IDs to devices
  // PDP
  //private final PowerDistributionPanel PDP = new PowerDistributionPanel(0);
  
  // Drive Motors
  private final WPI_VictorSPX driveA = new WPI_VictorSPX(2);
  private final WPI_VictorSPX driveB = new WPI_VictorSPX(4);
  private final WPI_VictorSPX driveC = new WPI_VictorSPX(13);
  private final WPI_VictorSPX driveD = new WPI_VictorSPX(14);
  
  // Belt Motors
  
  private final WPI_VictorSPX conv = new WPI_VictorSPX(3);
  private final WPI_VictorSPX bbar = new WPI_VictorSPX(15);

  // Lift Motors

  private final WPI_TalonFX lifta = new WPI_TalonFX(5);
  private final WPI_TalonFX liftb = new WPI_TalonFX(10);

  // Shooter Talons

  private final WPI_TalonSRX shoota = new WPI_TalonSRX(7);
  private final WPI_TalonSRX shootb = new WPI_TalonSRX(8);
  // Tilt Not here yet
  private final WPI_TalonFX tilt = new WPI_TalonFX(6);

  // END Attach CAN IDs to devices

  
  // Drive Functions
  private final DifferentialDrive vroom = new DifferentialDrive(driveA, driveB);

  // Controllers
  private final Joystick logA = new Joystick(0);
  private final Joystick xbox = new Joystick(1);
  private final Joystick Logi = new Joystick(2);

  // Talon FX Counting
  private final int unitsperRev = 2048 * 35;
  private final int numbRev = 5;

  // Other global variables

  double lift_backup = 0;
  double tilt_backup = 0;
  double bb = 0;
  double cc = 0;
  int senseX = 0;

  // Lift Limits - Encoder units
  int max_max  = 360000;
  int max_stop = 355000;
  int max_slow = 270000;
  int min_slow =  60000;
  int min_stop =  30000;
  int min_min  =   5000;
  double lift_speed = 0;

  double startTime;

  double currentDistance;

  UsbCamera camera0;
  UsbCamera camera1;
  VideoSink server0;
  VideoSink server1;
  NetworkTableEntry camera00_table;

  private static final double kValueToInches = 0.125/2.54;
  private final AnalogInput m_ultrasonic = new AnalogInput(0);


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {


    // Talon Encoder Stuff
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    lifta.configAllSettings(configs);
    liftb.configAllSettings(configs);

    lifta.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    lifta.setNeutralMode(NeutralMode.Brake);

    liftb.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    liftb.setNeutralMode(NeutralMode.Brake);

    liftb.setSensorPhase(true);

    lifta.configForwardSoftLimitEnable(true);
    lifta.configForwardSoftLimitThreshold(max_max);

    lifta.configReverseSoftLimitEnable(true);
    lifta.configReverseSoftLimitThreshold(min_min);
    
    liftb.configForwardSoftLimitEnable(true);
    liftb.configForwardSoftLimitThreshold(-1 * min_min);

    liftb.configReverseSoftLimitEnable(true);
    liftb.configReverseSoftLimitThreshold(-1*max_max);


    // Set up Voltage Compesation

    shoota.configVoltageCompSaturation(11);
    shoota.enableVoltageCompensation(true);
    shoota.configNeutralDeadband(0.04);
    shoota.setNeutralMode(NeutralMode.Coast);
    shootb.setSensorPhase(true);

    
    shootb.configVoltageCompSaturation(11);
    shootb.enableVoltageCompensation(true);
    shootb.configNeutralDeadband(0.04);
    shootb.setNeutralMode(NeutralMode.Coast);
    shootb.setSensorPhase(false);
    shootb.setInverted(true);

    // Set Controller Channels
    logA.setZChannel(4);
    logA.setThrottleChannel(3);
    logA.setTwistChannel(2);

    xbox.setZChannel(5);
    xbox.setThrottleChannel(2);
    xbox.setTwistChannel(3);

    Logi.setThrottleChannel(3);

    lifta.setInverted(false);
    lifta.setSensorPhase(false);

    liftb.setSensorPhase(true);


    //Cameras
    camera0 = CameraServer.getInstance().startAutomaticCapture();
    camera0.setResolution(640, 360);

    camera1 = CameraServer.getInstance().startAutomaticCapture();
    camera1.setResolution(640, 360);
    
    // camera1.setBrightness(1);
    // camera1.setExposureAuto();
  
    // Tilt
    tilt.setNeutralMode(NeutralMode.Brake);

    // Ultrasonic


  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    // Smart Dashboard Stuff Here
    SmartDashboard.putNumber("Y",logA.getY());
    SmartDashboard.putNumber("X",logA.getX());
    //SmartDashboard.putNumber("PDP Voltage", PDP.getVoltage());
    SmartDashboard.putNumber("Distance",cc);
    //SmartDashboard.putNumber("Position", lifta.getSelectedSensorPosition());
    SmartDashboard.putNumber("throttle channel", logA.getThrottle());
    SmartDashboard.putNumber("pulses", tilt_backup);
    SmartDashboard.putNumber("Falcon Encoder 5 pos", lifta.getSelectedSensorPosition());
    SmartDashboard.putNumber("Falcon Encoder 10 pos", liftb.getSelectedSensorPosition());

    currentDistance = m_ultrasonic.getValue() * kValueToInches;


  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    startTime = Timer.getFPGATimestamp();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
   double time = Timer.getFPGATimestamp();
  
    if (time-startTime < 3) {
      shoota.set(ControlMode.PercentOutput, .20);
      shootb.set(ControlMode.PercentOutput, -1 * .20);
    }
    else if (3 < time-startTime && time-startTime < 9) {
      shoota.set(ControlMode.PercentOutput, .20);
      shootb.set(ControlMode.PercentOutput, -1 * .20);
      bbar.set(ControlMode.PercentOutput,-0.6);
      conv.set(ControlMode.PercentOutput,.75);
    
    }

  //  else if (9 < time-startTime && time-startTime < 11){
  //    driveA.set(0.4);
  //    driveB.set(-0.4);
  //    driveC.follow(driveA);
  //    driveD.follow(driveB);
  //    shoota.set(ControlMode.PercentOutput, 0);
  //    shootb.set(ControlMode.PercentOutput, -1 * 0);
  //    bbar.set(ControlMode.PercentOutput,-0);
  //    conv.set(ControlMode.PercentOutput,0);
  // }
   else {
    driveA.set(0);
    driveB.set(0);
    driveC.follow(driveA);
    driveD.follow(driveB);
   }

  }

  /**
   * This function is called periodically during operator control.
   */
  
  @Override
  public void teleopInit() {
    
    lifta.setSelectedSensorPosition(0);
    liftb.setSelectedSensorPosition(0);
    
  }

  
   @Override
  public void teleopPeriodic() {

    // Drive
    
    double zoom = logA.getY() * .95;

    if (logA.getTwist() > 0.5) {
      zoom = -1 * logA.getTwist();
    }
    else if (logA.getThrottle() > 0.5) {
      zoom = logA.getThrottle();
    }


    double start_time = Timer.getFPGATimestamp();
    double start = 0;

    if(xbox.getRawButton(6) || logA.getRawButton(6)) { 
      vroom.arcadeDrive(0.4, -0.65);
      driveC.follow(driveA);
      driveD.follow(driveB);
    }
    else if (xbox.getRawButton(5) || logA.getRawButton(5)) {
      vroom.arcadeDrive(-0.4, 0.65);
      driveC.follow(driveA);
      driveD.follow(driveB);
    } 
    else {
      vroom.arcadeDrive(zoom, -1* logA.getZ());
      driveC.follow(driveA);
      driveD.follow(driveB);
    }


    // Conveyor
 

    final double bo = -0.60;
    final double co = 0.55;
    
    
    if (logA.getRawButtonPressed(1) || xbox.getRawButtonPressed(1)) {
      if (bb == 0)
      {
       bb = bo;
       cc = co;
      }
      else {
        bb = 0;
        cc = 0;
      }
    }
    else {
      if (logA.getRawButtonPressed(2) || xbox.getRawButtonPressed(2)) {
        if (bb == 0)
        {
         bb = bo;
        }
        else {
          bb = 0;
        }
      }
      if (logA.getRawButtonPressed(4) || xbox.getRawButtonPressed(4)) {
        if (cc == 0)
        {
          cc = co;
        }
        else {
          cc = 0;
        }
      }
    }

    if (xbox.getY() > 0.2 || xbox.getY() < -0.2) {
        bb = -1 * xbox.getY();
    }
    if (xbox.getZ() > 0.2 || xbox.getZ() < -0.2) {
        cc = -1 * xbox.getZ();
    }
    bbar.set(ControlMode.PercentOutput,bb);
    conv.set(ControlMode.PercentOutput,cc);

    // Coveyor Backup

    // bbar.set(ControlMode.PercentOutput, xbox.getY() * .55);
    // conv.set(ControlMode.PercentOutput, xbox.getZ() * .30); 


    // Lifter
    senseX = lifta.getSelectedSensorPosition(0);
    if (Logi.getRawButton(12)) {
      lift_speed = Logi.getTwist();
  }
  else {
    if (Logi.getRawButtonPressed(7)) {
      lift_speed = 0.11;
    }
    
    else if (Logi.getRawButtonPressed(8)) {
      lift_speed = -0.11;
    }

    if (lift_speed > 0.1) {
      if (senseX > (max_stop))
        lift_speed = 0;
      else if (senseX > (max_slow))
        lift_speed = .35;
      else
        lift_speed = .85;
    }
    else if (lift_speed < -0.1) {
      if (senseX < (min_stop))
        lift_speed = 0;
      else if (senseX < (min_slow))
        lift_speed = -.5;
      else
        lift_speed = -.95;
    }
  }

  if(Logi.getRawButton(11)) {
    lift_speed = 0;
  }



    lifta.set(ControlMode.PercentOutput, lift_speed);
    liftb.set(ControlMode.PercentOutput, -1 * lift_speed);

    // lifter backup

  //  lift_backup = -1 * xbox.getY();

  //  lifta.set(ControlMode.PercentOutput,lift_backup);

  //  liftb.set(ControlMode.PercentOutput, -1 * lift_backup);




    // Shooter

    double shootout = (Logi.getThrottle() + 1) / 2;

    shoota.set(ControlMode.PercentOutput, shootout);
    shootb.follow(shoota);

    // shoota.set(ControlMode.PercentOutput, xbox.getTwist());
    // shootb.set(ControlMode.PercentOutput, -1*xbox.getThrottle());

    // shootb.follow(shoota);
    // shootb.setInverted(true);

    // Tilt
    

    // tilt backup
    tilt_backup = xbox.getZ();
    int xpov = xbox.getPOV();
    int lpov = logA.getPOV();
    
    if (xpov == 315 || xpov == 0 || xpov == 45 || xpov ==360) {
        tilt.set(ControlMode.PercentOutput,0.1);
    } 
    else if (xpov == 180 || xpov == 135 || xpov == 225) {
        tilt.set(ControlMode.PercentOutput,-0.1);
    }
    else {
      tilt.set(ControlMode.PercentOutput,0);
    }



    // if (xbox.getRawButtonPressed(5))
    //   tilt_backup = -1 * tilt_backup;

    //tilt.set(ControlMode.PercentOutput,tilt_backup);

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    lift_backup = -1 * xbox.getY();

    lifta.set(ControlMode.PercentOutput,lift_backup);
    liftb.set(ControlMode.PercentOutput, -1 * lift_backup);

    lifta.setNeutralMode(NeutralMode.Coast);
    lifta.setNeutralMode(NeutralMode.Coast);
  }
}
