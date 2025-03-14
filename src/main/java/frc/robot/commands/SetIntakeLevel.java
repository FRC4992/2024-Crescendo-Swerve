// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeLevels;

public class SetIntakeLevel extends Command {
  /** Creates a new SetIntakeLevel. */

  IntakeLevels desiredLevel;

  public SetIntakeLevel(IntakeLevels desiredLevel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);

    this.desiredLevel = desiredLevel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(desiredLevel != null) {
      RobotContainer.intake.setLevel(desiredLevel);
    }
    //RobotContainer.intake.setRollerMotor(0.5);
    //RobotContainer.intake.setRotationMotor(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.stopRotationMotor(); // check
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
