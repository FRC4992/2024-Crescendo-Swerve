// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intakeRotationMotor;
  CANSparkMax intakeRollerMotor;

  RelativeEncoder intakeRotationEncoder;

  PIDController intakePID;

  DigitalInput topLimitSwitch;

  double calculatedSpeed;
  double currentPos;
  double setPoint;

  public Intake() {

    intakeRotationMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeRotationMotorID, MotorType.kBrushless);
    intakeRollerMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeRollerMotorID, MotorType.kBrushless);
    topLimitSwitch = new DigitalInput(Constants.IntakeConstants.kTopLimitSwitchID);

    intakeRotationEncoder = intakeRotationMotor.getEncoder();

    //intakeRotationMotor.setInverted(true);
    //intakeRollerMotor.setInverted(true);
  
    intakePID = new PIDController(Constants.IntakeConstants.kIntakeKP, Constants.IntakeConstants.kIntakeKI, Constants.IntakeConstants.kIntakeKP);
  
  }

  public boolean hasReachedMax() {
    return !topLimitSwitch.get();
  }

  public void resetEncoder() {
    // set encoder position to constant at top limit
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(hasReachedMax()) {
      resetEncoder(); 
    }

    calculatedSpeed = intakePID.calculate(currentPos, setPoint);
  }
}
