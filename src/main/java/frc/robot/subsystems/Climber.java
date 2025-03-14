// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonSRX climbMotorLeft;
  WPI_TalonSRX climbMotorRight;
  
  public final DigitalInput leftMagSensor;
  public final DigitalInput rightMagSensor;


  public Climber() {
  // initialize motors
  climbMotorLeft = new WPI_TalonSRX(ClimberConstants.kClimbMotorLeft);
  climbMotorRight = new WPI_TalonSRX(ClimberConstants.kClimbMotorRight);

  leftMagSensor = new DigitalInput(ClimberConstants.kLeftMagSensorPort);
  rightMagSensor = new DigitalInput(ClimberConstants.kRightMagSensorPort);

  }

  public boolean isBottomedOutLeft(){
    return !leftMagSensor.get();
  }

  public boolean isBottomedOutRight(){
    return !rightMagSensor.get();
  }

  public void extend() {
    climbMotorLeft.set(ClimberConstants.kClimbSpeed);
    climbMotorRight.set(ClimberConstants.kClimbSpeed);
  }
  
  public void stopClimbMotors() {
  climbMotorLeft.stopMotor();
  climbMotorRight.stopMotor();
  }

  public void retract() {
    if(!isBottomedOutLeft()) {
      climbMotorLeft.set(-ClimberConstants.kClimbSpeed);
    }
    else {
      climbMotorLeft.stopMotor();
    }
    if(!isBottomedOutRight()) {
      climbMotorRight.set(-ClimberConstants.kClimbSpeed);
    }
    else {
      climbMotorRight.stopMotor();
    }
  }

  public void setBrakeMode() {
    climbMotorLeft.setNeutralMode(NeutralMode.Brake);
    climbMotorRight.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
    climbMotorLeft.setNeutralMode(NeutralMode.Coast);
    climbMotorRight.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
