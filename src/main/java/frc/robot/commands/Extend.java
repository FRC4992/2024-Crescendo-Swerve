// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Extend extends Command {
  /** Creates a new Extend. */

  private double elapTime;

  public Extend() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elapTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (Timer.getFPGATimestamp() - elapTime <= 5){
      RobotContainer.climber.extend();
    //}
    //else {
      //RobotContainer.climber.stopClimbMotors();
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.stopClimbMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
