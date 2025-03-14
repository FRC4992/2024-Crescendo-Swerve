// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeStates;

public class ManualSetIntakeState extends Command {

  IntakeStates desiredState;

  /** Creates a new ManualSetIntakeLevel. */
  public ManualSetIntakeState(IntakeStates desiredState) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    this.desiredState = desiredState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(desiredState != null) {
      RobotContainer.intake.setState(desiredState);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setState(IntakeStates.ZERO);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
