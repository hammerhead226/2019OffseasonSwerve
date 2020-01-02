/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Waypoint.MotionType;

public class Robot extends TimedRobot {

  SwerveControl controller;
  Joystick joy = new Joystick(0);

  private CANSparkMax drive1;
  private CANSparkMax steer1;
  private AnalogInput encoder1;
  private SwerveModule module1;

  private CANSparkMax drive2;
  private CANSparkMax steer2;
  private AnalogInput encoder2;
  private SwerveModule module2;

  private CANSparkMax drive3;
  private CANSparkMax steer3;
  private AnalogInput encoder3;
  private SwerveModule module3;

  private CANSparkMax drive4;
  private CANSparkMax steer4;
  private AnalogInput encoder4;
  private SwerveModule module4;

  private PigeonIMU pigeonIMU;

  private boolean toggle = false;
  private int finishedPoints = 0;
  private Waypoint[] points = {
    new Waypoint(20, 20),
    new Waypoint(-90),
    new Waypoint(-40, 20),
    new Waypoint(40, 60),
    new Waypoint(90),
    new Waypoint(-20, -100),
    new Waypoint(0)
  };
  
  @Override
  public void robotInit() {

    drive1 = new CANSparkMax(RobotMap.DRIVE_1, MotorType.kBrushless);
    drive1.getEncoder().setPosition(0);
    steer1 = new CANSparkMax(RobotMap.STEER_1, MotorType.kBrushless);
    drive1.setIdleMode(IdleMode.kBrake);
    steer1.setIdleMode(IdleMode.kBrake);
    encoder1 = new AnalogInput(RobotMap.ENCODER_1);
    steer1.setInverted(true);
    module1 = new SwerveModule(drive1, steer1, encoder1, 1);

    drive2 = new CANSparkMax(RobotMap.DRIVE_2, MotorType.kBrushless);
    drive2.getEncoder().setPosition(0);
    steer2 = new CANSparkMax(RobotMap.STEER_2, MotorType.kBrushless);
    drive2.setIdleMode(IdleMode.kBrake);
    steer2.setIdleMode(IdleMode.kBrake);
    steer2.setInverted(true);
    encoder2 = new AnalogInput(RobotMap.ENCODER_2);
    module2 = new SwerveModule(drive2, steer2, encoder2, 2);

    drive3 = new CANSparkMax(RobotMap.DRIVE_3, MotorType.kBrushless);
    drive3.getEncoder().setPosition(0);
    steer3 = new CANSparkMax(RobotMap.STEER_3, MotorType.kBrushless);
    steer3.setInverted(true);
    drive3.setIdleMode(IdleMode.kBrake);
    steer3.setIdleMode(IdleMode.kBrake);
    encoder3 = new AnalogInput(RobotMap.ENCODER_3);
    module3 = new SwerveModule(drive3, steer3, encoder3, 3);

    drive4 = new CANSparkMax(RobotMap.DRIVE_4, MotorType.kBrushless);
    drive4.getEncoder().setPosition(0);
    steer4 = new CANSparkMax(RobotMap.STEER_4, MotorType.kBrushless);
    steer4.setInverted(true);
    drive4.setIdleMode(IdleMode.kBrake);
    steer4.setIdleMode(IdleMode.kBrake);
    encoder4 = new AnalogInput(RobotMap.ENCODER_4);
    module4 = new SwerveModule(drive4, steer4, encoder4, 4);

    joy = new Joystick(RobotMap.JOYSTICK);

    pigeonIMU = new PigeonIMU(new TalonSRX(RobotMap.PIGEON));
    pigeonIMU.setYaw(0);

    controller = new SwerveControl(module1, module2, module3, module4, pigeonIMU);
  }

  
  @Override
  public void robotPeriodic() {
  }

  
  @Override
  public void autonomousInit() {
   
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopPeriodic() {

    double x = (Math.abs(joy.getRawAxis(0)) < 0.2) ? 0 : 0.5 * joy.getRawAxis(0);
    double y = (Math.abs(joy.getRawAxis(1)) < 0.2) ? 0 : -0.5 * joy.getRawAxis(1);
    double rotate = (Math.abs(joy.getRawAxis(4)) < 0.1) ? 0 : joy.getRawAxis(4) * 0.5;

    if(joy.getRawButton(1)) {
      toggle = true;
    } else if(joy.getRawButton(2)) {
      toggle = false;
      finishedPoints = 0;
    }

    if(toggle) {

      Waypoint currentPoint = points[finishedPoints];
      if(currentPoint.motionType == MotionType.LINEAR) controller.toPosition(currentPoint.x, currentPoint.y);
      if(currentPoint.motionType == MotionType.ROTATION) controller.toAngle(currentPoint.theta);

      if(controller.isDone()) {
        finishedPoints++;
        if(finishedPoints == points.length) {
          finishedPoints--;
        }
      }
    } else {
      controller.control(x, y, rotate);
    }

    if(joy.getRawButton(8)) {
      pigeonIMU.setYaw(0);
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledPeriodic() { 
  }
}
