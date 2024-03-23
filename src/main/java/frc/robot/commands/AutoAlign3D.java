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

public class AutoAlign3D extends Command {
  /** Creates a new AutoAlign3D. */
  PIDController latitudinalController = new PIDController(0.1, 0, 0);
  PIDController longitudinalController = new PIDController(0.1, 0, 0);
  PIDController rotationalController = new PIDController(0.05, 0, 0);

  public AutoAlign3D() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerve);

    rotationalController.setTolerance(2.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] array = RobotContainer.getLimelight3DPose();

    double latitudinalMeasurement = array[0];
    double longitudinalMeasurement = array[1];
    double rotationalMeasurement = array[4];

    double latitudinalSpeed = latitudinalController.calculate(latitudinalMeasurement, 0);
    double longitudinalSpeed = longitudinalController.calculate(longitudinalMeasurement, 0.92);
    double rotationalSpeed = rotationalController.calculate(rotationalMeasurement, 0);

    if(RobotContainer.getLimelightFoundTarget()) {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(latitudinalSpeed, longitudinalSpeed, rotationalSpeed);
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
