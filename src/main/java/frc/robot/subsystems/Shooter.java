// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  WPI_TalonSRX shootMotor;
  WPI_TalonSRX feedMotor;

  public Shooter() {
    shootMotor = new WPI_TalonSRX(ShooterConstants.kShooterMotorID);
    feedMotor = new WPI_TalonSRX(ShooterConstants.kFeedMotorID);
  }

  public void shoot() {
    shootMotor.set(ShooterConstants.kShooterSpeed);
  }

  public void stopShootMotor() {
    shootMotor.stopMotor();
  }

  public void feed() {
    feedMotor.set(ShooterConstants.kFeedSpeed);
  }
  
  public void stopFeedMotor() {
    feedMotor.stopMotor();
  }

  public void intake() {
    shootMotor.set(-ShooterConstants.kShooterSpeed);
    //shootMotor.set(-0.9);
    feedMotor.set(-ShooterConstants.kFeedSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
