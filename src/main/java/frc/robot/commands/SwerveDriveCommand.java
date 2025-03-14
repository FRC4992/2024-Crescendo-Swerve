// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveCommand extends Command {

  SwerveDrive swerve;
  //Supplier<Double> xSpdFunction, ySpdFunction, rotationSpdFunction;
  double xSpdFunction, ySpdFunction, rotationSpdFunction;
  //Supplier<Boolean> fieldOrientedFunction;
  boolean fieldOrientedFunction;

  // Rate limiter variables
  SlewRateLimiter xLimiter, yLimiter, rotationLimiter;

  CommandXboxController controller;

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(SwerveDrive swerve, 
    CommandXboxController controller,
    // double xSpdFunction, double ySpdFunction,
    // double rotationSpdFunction, 
    boolean fieldOrientedFunction) {

    this.controller = controller;

    this.swerve = swerve;
    // this.xSpdFunction = xSpdFunction;
    // this.ySpdFunction = ySpdFunction;
    // this.rotationSpdFunction = rotationSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;

    // Rate limitiers - add to constants if needed
    //this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.TELEOP_MAX_ACCELERATION_UNITS_PER_SEC);
    //this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.TELEOP_MAX_ACCELERATION_UNITS_PER_SEC);
    //this.rotationLimiter = new SlewRateLimiter(Constants.DriveConstants.TELEOP_MAX_ANGULAR_ACCELERATION_UNITS_PER_SEC);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("command running");
    //System.out.println(RobotContainer.swerve.FLModule.getState());
    //System.out.println(RobotContainer.m_driverController.getLeftX());

    double xSpeed = controller.getLeftY();
    double ySpeed = controller.getLeftX();
    double rotationSpeed = -controller.getRightX();
    // double xSpeed = xSpdFunction;
    // double ySpeed = ySpdFunction;
    // double rotationSpeed = rotationSpdFunction;

    // For deadband
    // xSpeed = Math.abs(xSpeed) > OperatorConstants.kDeadbandLeftStick ? xSpeed : 0.0;
    // ySpeed = Math.abs(ySpeed) > OperatorConstants.kDeadbandLeftStick ? ySpeed : 0.0;
    // rotationSpeed = Math.abs(rotationSpeed) > OperatorConstants.kDeadbandRightStick ? rotationSpeed : 0.0;

    //Haashim deadband logic here

    if(Math.abs(xSpeed) < OperatorConstants.kDeadbandLeftStick){
      xSpeed = 0;
    }
    else{
      xSpeed = (1/(1 - OperatorConstants.kDeadbandLeftStick)) * (xSpeed + (-Math.signum(xSpeed) * OperatorConstants.kDeadbandLeftStick));
    }

    if(Math.abs(ySpeed) < OperatorConstants.kDeadbandLeftStick){
      ySpeed = 0;
    }
    else{
      ySpeed = (1/(1 - OperatorConstants.kDeadbandLeftStick)) * (ySpeed + (-Math.signum(ySpeed) * OperatorConstants.kDeadbandLeftStick));
    }

    if(Math.abs(rotationSpeed) < OperatorConstants.kDeadbandRightStick){
      rotationSpeed = 0;
    }
    else{
      rotationSpeed = (1/(1 - OperatorConstants.kDeadbandRightStick)) * (rotationSpeed + (-Math.signum(rotationSpeed) * OperatorConstants.kDeadbandRightStick));
    }

    // Rate limiter
    //xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;
    //ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;
    //rotationSpeed = rotationLimiter.calculate(rotationSpeed) * Constants.DriveConstants.TELEOP_MAX_ANGULAR_SPEED_RAD_PER_SEC;

    ChassisSpeeds chassisSpeeds;
    if(fieldOrientedFunction) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, swerve.getRotation2d());
    }
    else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
      //System.out.println(chassisSpeeds);
    }

    SwerveModuleState[] moduleStates = DriveConstants.SWERVE_DRIVE_KINEMATIC.toSwerveModuleStates(chassisSpeeds);

    swerve.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
