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
  
  public final DigitalInput topLimitSwitch;
  public final DigitalInput bottomLimitSwitch;

  public static RelativeEncoder climberEncoder;
  public final PIDController pid;
  public static double computedSpeed;
    public static double setPoint;

  public Climber() {
  // initialize motors
  climbMotorLeft = new WPI_TalonSRX(Constants.ClimberConstants.kClimbMotorLeft);
  climbMotorRight = new WPI_TalonSRX(Constants.ClimberConstants.kClimbMotorRight);

  topLimitSwitch = new DigitalInput(Constants.OperatorConstants.kTopLimitSwitchPort);
  bottomLimitSwitch = new DigitalInput(Constants.OperatorConstants.kBottomLimitSwitchPort);

  pid = new PIDController(Constants.ClimberConstants.kClimberKP, Constants.ClimberConstants.kClimberKI, Constants.ClimberConstants.kClimberKD);
  pid.setTolerance(3); // change as needed
  }

  public boolean isBottomedOut(){
    return !bottomLimitSwitch.get();
  }

  public boolean isToppedOut(){
    return !topLimitSwitch.get();
  }

  public void extend() {
  // speeds can be changed to use pid computedSpeed variable if needed
    // setPoint = 10;
    if(isToppedOut()) {
      stopClimbMotor();
    } else {
    climbMotorLeft.set(Constants.ClimberConstants.kClimbSpeed);
    climbMotorRight.set(-Constants.ClimberConstants.kClimbSpeed); // check if this is right spot for negative
    }
  }
  
  public void stopClimbMotor() {
  climbMotorLeft.stopMotor();
  climbMotorRight.stopMotor();
  }

  public void retract() {
  // speeds can be changed to use pid computedSpeed variable if needed
    // setPoint = 0;
    if(isBottomedOut()) {
      stopClimbMotor();
    } else {
    climbMotorLeft.set(-Constants.ClimberConstants.kClimbSpeed); // check
    climbMotorRight.set(Constants.ClimberConstants.kClimbSpeed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    computedSpeed = pid.calculate(climberEncoder.getPosition(), setPoint);
  }
}
