// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.SetIntakeState;
import frc.robot.subsystems.Intake.IntakeStates;;

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
    shootMotor.set(-ShooterConstants.kShooterSpeed/2);
    feedMotor.set(-ShooterConstants.kFeedSpeed/2);
  }

  public Command getSpeakerShootCommand() {
    return new InstantCommand(this::shoot, this)
      .andThen(
        new InstantCommand(this::feed, this))
      .andThen(
        new WaitCommand(1.75))
      .andThen(
        new SetIntakeState(IntakeStates.BOUNCE))
      .andThen(
        new WaitCommand(0.20))
      .andThen(
        new SetIntakeState(IntakeStates.FEED))
      .andThen(
        new WaitCommand(0.75))
      .andThen(
        new SetIntakeState(IntakeStates.ZERO))
      .andThen(
        new InstantCommand(this::stopShootMotor, this))
      .andThen(
        new InstantCommand(this::stopFeedMotor, this));  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
