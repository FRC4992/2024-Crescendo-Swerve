// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {

  public SwerveModule FLModule = new SwerveModule(
    DriveConstants.kFLDriveMotorID, 
    DriveConstants.kFLRotationMotorID, 
    DriveConstants.kFLCancoderID, 
    false, 
    true, 
    DriveConstants.kFLOffset, // abs enc offset, add to constants, paramter here
    false); // abs enc reversed, add to constants, parameter here

  SwerveModule FRModule = new SwerveModule(
    DriveConstants.kFRDriveMotorID, 
    DriveConstants.kFRRotationMotorID, 
    DriveConstants.kFRCancoderID, 
    true, 
    true, 
    DriveConstants.kFROffset, // abs enc offset, add to constants, paramter here
    false); // abs enc reversed, add to constants, parameter here

  SwerveModule BLModule = new SwerveModule(
    DriveConstants.kBLDriveMotorID, 
    DriveConstants.kBLRotationMotorID, 
    DriveConstants.kBLCancoderID, 
    false, 
    true, 
    DriveConstants.kBLOffset, // abs enc offset, add to constants, paramter here
    false); // abs enc reversed, add to constants, parameter here

  SwerveModule BRModule = new SwerveModule(
    DriveConstants.kBRDriveMotorID, 
    DriveConstants.kBRRotationMotorID, 
    DriveConstants.kBRCancoderID, 
    true, 
    true, 
    DriveConstants.kBROffset, // abs enc offset, add to constants, paramter here
    false); // abs enc reversed, add to constants, parameter here

  AHRS navx = new AHRS(SPI.Port.kMXP);

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {


    // for field-oriented swerve, resets navx on robot init
    new Thread(() -> { // new thread, doesn't interfere with anything else
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {}
    }).start();
  }

  public void zeroHeading() {
    navx.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360); // clamps value within -180 to 180 deg
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // double loggingState[] = {

    // };

    //SmartDashboard.putNumber("Robot Heading", getHeading());

  }

  public void stopModules() {
    FLModule.stop();
    FRModule.stop();
    BLModule.stop();
    BRModule.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSec);
    FLModule.setDesiredState(desiredStates[0]);
    FRModule.setDesiredState(desiredStates[1]);
    BLModule.setDesiredState(desiredStates[2]);
    BRModule.setDesiredState(desiredStates[3]);
  }

}
