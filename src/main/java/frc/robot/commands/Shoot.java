// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Shoot extends Command {
  /** Creates a new Shoot. */

  double timer;
  double endTimer;

  boolean hasShot = false;

  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = Timer.getFPGATimestamp();
    hasShot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.shoot();
    // System.out.println(Timer.getFPGATimestamp() - timer);
    if(Timer.getFPGATimestamp() - timer >= 1.5 && !hasShot) {
      // System.out.println("feeding");
      RobotContainer.shooter.feed();
      endTimer = Timer.getFPGATimestamp();
      hasShot=true;
    }
    if(Timer.getFPGATimestamp() - timer >= 2.5 && hasShot) {
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stopShootMotor();
    RobotContainer.shooter.stopFeedMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
