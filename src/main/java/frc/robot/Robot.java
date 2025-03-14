// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake.IntakeLevels;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // reset 

    // UsbCamera camera = new UsbCamera("cam0", 0);
    // camera.setFPS(15);
    // camera.setResolution(320, 420);
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());

    //System.out.println(RobotContainer.intake.noteLoaded());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.intake.setCoastMode();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // check:
    // CommandScheduler.getInstance().cancelAll();
    RobotContainer.swerve.resetEncoders();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // SmartDashboard.putNumber("FLModule:", RobotContainer.swerve.FLModule.getDistancePosition());
    // SmartDashboard.putNumber("FRModule:", RobotContainer.swerve.FRModule.getDistancePosition());
    // SmartDashboard.putNumber("BLModule:", RobotContainer.swerve.BLModule.getDistancePosition());
    // SmartDashboard.putNumber("BRModule:", RobotContainer.swerve.BRModule.getDistancePosition());

    // System.out.println(RobotContainer.swerve.FLModule.getPosition());
    // System.out.println(RobotContainer.swerve.FRModule.getPosition());
    // System.out.println(RobotContainer.swerve.BLModule.getPosition());
    // System.out.println(RobotContainer.swerve.BRModule.getPosition());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    RobotContainer.swerve.resetEncoders();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.intake.setLevel(IntakeLevels.STOWED);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // SmartDashboard.putNumber("FLModule:", RobotContainer.swerve.FLModule.getDistancePosition());
    // SmartDashboard.putNumber("FRModule:", RobotContainer.swerve.FRModule.getDistancePosition());
    // SmartDashboard.putNumber("BLModule:", RobotContainer.swerve.BLModule.getDistancePosition());
    // SmartDashboard.putNumber("BRModule:", RobotContainer.swerve.BRModule.getDistancePosition());
    // SmartDashboard.putNumber("nav", RobotContainer.swerve.getHeading());
    //System.out.println(RobotContainer.swerve.navx.getRotation2d());
    //System.out.println("Absolute Encoder: " + RobotContainer.swerve.FLModule.getAbsoluteEncoderDeg());
    //System.out.println("Left mag sensor: " + RobotContainer.climber.isBottomedOutLeft());
    //System.out.println("Right mag sensor: " + RobotContainer.climber.isBottomedOutRight());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
