// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoAlign extends Command {

  PIDController rotationalPID = new PIDController(0.01, 0, 0);
  PIDController translationalPID = new PIDController(0.1, 0, 0);

  /** Creates a new AutoAlign. */
  public AutoAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerve);

    rotationalPID.setTolerance(1);
    translationalPID.setTolerance(0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xMeasurement = RobotContainer.getLimelightX();
    double xSpeed = rotationalPID.calculate(xMeasurement, 0);

    double yMeasurement = RobotContainer.getLimelightArea();
    double ySpeed = translationalPID.calculate(yMeasurement, 1.02);

    if(RobotContainer.getLimelightFoundTarget()) {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-ySpeed, 0, xSpeed);
      SwerveModuleState[] moduleStates = Constants.DriveConstants.SWERVE_DRIVE_KINEMATIC.toSwerveModuleStates(chassisSpeeds);
      RobotContainer.swerve.setModuleStates(moduleStates);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
