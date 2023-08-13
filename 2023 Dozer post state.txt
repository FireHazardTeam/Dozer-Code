// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.time.StopWatch;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.sendable.SendableBuilder.BackendKind;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SerialPort.Port;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

 

  //private DoubleSolenoid pistonKicker;
  private boolean pistonKicked = false;
  //private Joystick stick;
  private int leftJoystick = 0;
  Joystick stick = new Joystick(leftJoystick);	

  DoubleSolenoid pistonKicker = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

//motor controller declarations and specifications
  MotorController m_frontleft = new PWMVictorSPX(1);
  MotorController m_backleft = new PWMVictorSPX(2);
  MotorController m_frontright = new PWMVictorSPX(3);
  MotorController m_backright = new PWMVictorSPX(4);
  MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_frontleft, m_backleft);
  MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_frontright, m_backright);

  DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);

  private int driveDirection = 1;   // 1 for forward, -1 for backward
  
  //Xbox controller statements
  private final XboxController m_controller = new XboxController(0);
  private int buttonA = 1;
  private int buttonB = 2;
  private int buttonX = 3;
  private int buttonY = 4;
  
  private final Timer m_timer = new Timer();  // create timer and set to 0

  // try { AHRS ahrs = new AHRS(Port.kUSB);
    private AHRS ahrs = null;
    
  {  
 
}
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */


  //String trajectoryJSON = "paths/DriverStation1.wpilib.json";
  //Trajectory trajectory = new Trajectory();


  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);

    //controls the robot speed
    m_leftDrive.set(1);
    m_rightDrive.set(1);

    //AHRS ahrs = new AHRS(Port.kMXP);

    ahrs = null;    // use for gyro on USB port
    try{
      ahrs = new AHRS(Port.kUSB);
      SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());

    } catch (Exception ex) {
      SmartDashboard.putString("FirmwareVersion", "navX not connected");
      DriverStation.reportError("Error creating navX:  " + ex.getMessage(), true);
    }

    Shuffleboard.getTab("Gyro").add(ahrs);
    //CameraServer.startAutomaticCapture();
       
    }
    




  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();      // set timer to 0
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {


    //try {
      //m_robotDrive.arcadeDrive(0.5, 0.0, false);
      //Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      //trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   // } catch (IOException ex) {
      //DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    
   // }}

   //kick out game piece
   pistonKicker.set(Value.kForward);
   if(m_timer.get() >= .75)
  pistonKicker.set(Value.kReverse);
   // Drive for seconds
   if (m_timer.get() <= 1)  m_robotDrive.stopMotor();
 else {
     if (m_timer.get() <= 2.62) { //auto dock

     //if (m_timer.get() <= 2.85) { //cable side
      
      // Drive forwards half speed
       m_robotDrive.arcadeDrive(-0.5, 0.0, false);
    } else {
      if (m_timer.get() <= 2.63) m_robotDrive.stopMotor();
      else {
        if (m_timer.get() <= 2.64) m_robotDrive.arcadeDrive(0.5, 0.0, false);
      else {      
        if (m_timer.get() <= 2.72);
       m_robotDrive.stopMotor(); // stop robot
     }
    }}}
     
    }
  
    //Shoot piston function. Not currently being utilized.
    private void shootPiston(){

      System.out.println("shootPiston() activated");
  
      boolean buttonPressed = stick.getRawButtonReleased(buttonB);
      
      //pistonKicker.toggle();  // toggle doesn't seem to work
  
      pistonKicker.set(Value.kForward);
  
      if (buttonPressed == true)
      {
        pistonKicker.toggle(); 
        
        m_timer.reset();
        pistonKicker.set(kForward);
        if (m_timer.get() <= .75);
        
          pistonKicker.set(kReverse);
      }
      else{
        System.out.println("Controller value " + m_controller.getName());
      }
  
      pistonKicker.set(Value.kOff);
      pistonKicker.close();
    }
  
    public void debugPistonKicked() {
      
      if (pistonKicked == true) {
        SmartDashboard.putString("DB/String 8", "Piston kicked"); 
      }
      else {
        SmartDashboard.putString("DB/String 8", "Piston not kicked");
      }
      
    } 
  
    private void driveIt() {
      double xAxis = stick.getRawAxis(1);  //left joystick - forward / back
      double yAxis = stick.getRawAxis(4);  //right joystick - left / right 
    }
  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {    
    pistonKicker.toggle();

    pistonKicked = true;
    System.out.println("Piston kicker initialized");
    pistonKicker.set(Value.kReverse);
    //pistonKicker.set(Value.kOff);
    debugPistonKicked();
  }

  /** This function is called periodically during teleoperated mode. 
   * @param value 
   * @param key 
   * @return 
   * @return */
  @Override
  public void teleopPeriodic() {
  //public void teleopPeriodic(Object value, String key) {
  

      double xAxis = stick.getRawAxis(1);  //left joystick - forward / back
    double yAxis = stick.getRawAxis(4);  //right joystick - left / right
   
    boolean buttonPressedB = stick.getRawButtonPressed(buttonB);
    boolean buttonPressedA = stick.getRawButtonReleased(buttonA);
    boolean buttonPressedX = stick.getRawButtonPressed(buttonX);
    boolean buttonPressedY = stick.getRawButtonPressed(buttonY);
    m_robotDrive.arcadeDrive(driveDirection * -m_controller.getLeftY(), 
                             driveDirection * -m_controller.getRightX());
    //buttonY = (-1 *valueOf(leftJoystick));


    //try { AHRS ahrs = new AHRS(Port.kUSB);

  

    float fYaw;
    float fPitch;
    float fRoll; 
    
    fYaw = ahrs.getYaw();
    fPitch = ahrs.getPitch();
    fRoll = ahrs.getRoll();
      //"SmartDashboard.putString("DB/String 8", value: "Piston not kicked");
      SmartDashboard.putNumber("Yaw", 0);
      SmartDashboard.putNumber("Pitch", 1); 
      SmartDashboard.putNumber("Roll", 10); 

      SmartDashboard.putNumber("fYaw", fYaw);
      SmartDashboard.putNumber("fPitch", fPitch); 
      SmartDashboard.putNumber("fRoll", fRoll); 

      SmartDashboard.putBoolean( "IMU_Connected",   ahrs.isConnected());
      SmartDashboard.putNumber( "IMU_Yaw",   ahrs.getYaw());
      SmartDashboard.putNumber("IMU_Pitch",  ahrs.getPitch());
      SmartDashboard.putNumber("IMU_Roll",   ahrs.getRoll());
    

    if (buttonPressedY == true)
    {
      driveDirection = driveDirection * -1;
    }


    //piston controls
      if (buttonPressedA == true)
      {
        pistonKicker.toggle();
      }
      if (buttonPressedB == true)
      {
        pistonKicker.toggle(); 
        
        pistonKicker.set(kForward);
        //pistonKicker.set(kReverse);
      }
      if(buttonPressedX == true){
        pistonKicker.set(kReverse);}
        
      }
    
    
    //Tank drive with a given left and right rates
    //m_robotDrive.tankDrive(-m_controller.getLeftY(), -m_controller.getRightX());

    // Arcade drive with a given forward and turn rate
    //m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());

    // Curvature drive with a given forward and turn rate, as well as a button for turning in-place.
    //m_robotDrive.curvatureDrive(-m_controller.getLeftY(), -m_controller.getRightX(), m_controller.getButton(1));
  
  
  
  int valueOf(int leftJoystick2) {
    return 0;
  }




  int getRawButtonPressed(int buttonY2) {
    return 0;
  }




  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private void testDriveDirection() {}
    //invert direction of the robot from opening









  public MotorController getM_frontleft() {
    return m_frontleft;
  }






  public void setM_frontleft(MotorController m_frontleft) {
    this.m_frontleft = m_frontleft;
  }






  public MotorController getM_backleft() {
    return m_backleft;
  }

    




  public void setM_backleft(MotorController m_backleft) {
    this.m_backleft = m_backleft;
  }






  public MotorController getM_frontright() {
    return m_frontright;
  }






  public void setM_frontright(MotorController m_frontright) {
    this.m_frontright = m_frontright;
  }






  public MotorController getM_backright() {
    return m_backright;
  }






  public void setM_backright(MotorController m_backright) {
    this.m_backright = m_backright;
  }






  public MotorControllerGroup getM_leftDrive() {
    return m_leftDrive;
  }






  public void setM_leftDrive(MotorControllerGroup m_leftDrive) {
    this.m_leftDrive = m_leftDrive;
  }






  public MotorControllerGroup getM_rightDrive() {
    return m_rightDrive;
  }






  public void setM_rightDrive(MotorControllerGroup m_rightDrive) {
    this.m_rightDrive = m_rightDrive;
  }






  public DifferentialDrive getM_robotDrive() {
    return m_robotDrive;
  }






  public void setM_robotDrive(DifferentialDrive m_robotDrive) {
    this.m_robotDrive = m_robotDrive;
  }






  public XboxController getM_controller() {
    return m_controller;
  }






  public int getButtonA() {
    return buttonA;
  }






  public void setButtonA(int buttonA) {
    this.buttonA = buttonA;
  }






  public int getButtonB() {
    return buttonB;
  }






  public void setButtonB(int buttonB) {
    this.buttonB = buttonB;
  }






  public int getButtonX() {
    return buttonX;
  }






  public void setButtonX(int buttonX) {
    this.buttonX = buttonX;
  }






  public int getButtonY() {
    return buttonY;
  }






  public void setButtonY(int buttonY) {
    this.buttonY = buttonY;
  }






  public Timer getM_timer() {
    return m_timer;
  }






  /*public DoubleSolenoid getPistonKicker() {
    return pistonKicker;
  }






  public void setPistonKicker(DoubleSolenoid pistonKicker) {
    this.pistonKicker = pistonKicker;
  }*/
}
