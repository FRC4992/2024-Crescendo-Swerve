// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonSRX climbMotorLeft;
  WPI_TalonSRX climbMotorRight;
  
  public final DigitalInput leftMagSensor;
  public final DigitalInput rightMagSensor;


  public Climber() {
  // initialize motors
  climbMotorLeft = new WPI_TalonSRX(Constants.ClimberConstants.kClimbMotorLeft);
  climbMotorRight = new WPI_TalonSRX(Constants.ClimberConstants.kClimbMotorRight);

  leftMagSensor = new DigitalInput(Constants.ClimberConstants.kClimbMotorLeft);
  rightMagSensor = new DigitalInput(Constants.ClimberConstants.kClimbMotorRight);

  }

  public boolean isBottomedOutLeft(){
    return !leftMagSensor.get();
  }

  public boolean isBottomedOutRight(){
    return !rightMagSensor.get();
  }

  public void extend() {
    climbMotorLeft.set(Constants.ClimberConstants.kClimbSpeed);
    climbMotorRight.set(Constants.ClimberConstants.kClimbSpeed);
  }
  
  public void stopClimbMotors() {
  climbMotorLeft.stopMotor();
  climbMotorRight.stopMotor();
  }

  public void retract() {
    if (!isBottomedOutLeft()){
      climbMotorLeft.set(-Constants.ClimberConstants.kClimbSpeed);
      climbMotorRight.set(Constants.ClimberConstants.kClimbSpeed);
    }
    else{
      stopClimbMotors();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
