// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
  
     private XRPMotor leftMotor1 = new XRPMotor(0);
     private XRPMotor rightMotor1 = new XRPMotor(1);

     private DifferentialDrive dDrive = new DifferentialDrive(leftMotor1, rightMotor1);
  
     private XboxController joy1 = new XboxController(0);

     // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

      
     
      private final double kDriveTick2Feet = 1.0 /128*6*Math.PI/12;


  public Robot() {}



  @Override
  public void autonomousInit() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();

  
  }

  final double kP = 0.05;

 double setpoint = 0;
  @Override
  public void autonomousPeriodic() {

      if (joy1.getRawButton(1));{
        setpoint = 10;
    } if (joy1.getRawButton(2));{
        setpoint = 0;   
    }
  

    double sensorPosition = m_leftEncoder.get() * kDriveTick2Feet;
    double sensorPosition2 = m_rightEncoder.get() * kDriveTick2Feet;

    double error = setpoint - sensorPosition;
    double error2 = setpoint - sensorPosition2;

    double outputSpeed = kP * error;
    double outputSpeed2 = kP  *error2;

    leftMotor1.set(-outputSpeed);
    rightMotor1.set(outputSpeed2);
  }

  @Override
  public void robotPeriodic(){
   SmartDashboard.putNumber("leftEncoder value", m_leftEncoder.get());
   SmartDashboard.putNumber("rightEncoder value", m_rightEncoder.get());
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    double speed = -joy1.getLeftY();
    double speed2 = joy1.getRightY();
   

    dDrive.tankDrive(speed, speed2);

  }
  

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
