// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoAlign3D;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Extend;
import frc.robot.commands.IntakeFromShooter;
import frc.robot.commands.ManualIntakeSpeed;
import frc.robot.commands.ManualSetIntakeState;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.Retract;
import frc.robot.commands.SetIntakeLevel;
import frc.robot.commands.SetIntakeState;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Intake.IntakeLevels;
import frc.robot.subsystems.Intake.IntakeStates;

import java.sql.Driver;
import java.util.List;

import javax.swing.border.EtchedBorder;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static SwerveDrive swerve = new SwerveDrive();
  public static Shooter shooter = new Shooter();
  public static Climber climber = new Climber();
  public static Intake intake = new Intake();

  private final SendableChooser<Command> autoChooser;

  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public static NetworkTableEntry tx = table.getEntry("tx");
  public static NetworkTableEntry ta = table.getEntry("ta");
  public static NetworkTableEntry tv = table.getEntry("tv");
  public static NetworkTableEntry t6t_cs = table.getEntry("t6t_cs");

  public static double getLimelightX() {
    return tx.getDouble(0.0);
  }
  public static double getLimelightArea() {
    return ta.getDouble(0.0);
  }
  public static boolean getLimelightFoundTarget() {
    return tv.getDouble(0) == 1;
  }

  public static double[] getLimelight3DPose() {
    return t6t_cs.getDoubleArray(new double[]{0.0});
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    NamedCommands.registerCommand("ShootCommand", shooter.getSpeakerShootCommand());
    NamedCommands.registerCommand("Intake", intake.autoIntakeAndLoadCommand());

    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, m_driverController, true));
    //swerve.setDefaultCommand(new SwerveDriveCommand(swerve, m_driverController, 0.5, 0, 0, false));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // For field-oriented swerve, reset heading on button press

    m_driverController.leftBumper().whileTrue(new IntakeFromShooter());
    //m_driverController.rightBumper().onTrue(new Shoot());
    m_driverController.rightBumper().onTrue(shooter.getSpeakerShootCommand());

    //m_driverController.y().whileTrue(new AutoAlign());
    m_driverController.y().whileTrue(new AutoAlign3D());
    m_driverController.povLeft().onTrue(new ResetHeading());

    //m_driverController.b().onTrue(new SetIntakeLevel(IntakeLevels.GROUND));
    m_driverController.b().onTrue(intake.intakeAndLoadCommand());
    m_driverController.a().onTrue(intake.getAmpShootCommand());
    m_driverController.x().onTrue(new SetIntakeLevel(IntakeLevels.STOWED));
    // m_driverController.x().onTrue(new InstantCommand(
    //   () -> CommandScheduler.getInstance().cancelAll()
    // ));
    
    //m_driverController.a().onTrue(new SetIntakeLevel(IntakeLevels.AMP)); // 
    m_driverController.povDown().whileTrue(new ManualSetIntakeState(IntakeStates.INTAKE)); // 
    //m_driverController.povDown().whileTrue(new ManualIntakeSpeed());

    m_driverController.povUp().whileTrue(new ManualSetIntakeState(IntakeStates.FEED)); // 
    m_driverController.povRight().whileTrue(new ManualSetIntakeState(IntakeStates.EJECT)); // 
    m_driverController.back().whileTrue(new Retract());
    m_driverController.start().whileTrue(new Extend());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    //PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath");
    // SmartDashboard.putString("Path Poses", path.getPathPoses().toString());
    // SmartDashboard.putString("Path Points", path.getAllPathPoints().toString());

    // return new SequentialCommandGroup( // test (1e) for single paths:
    //   new InstantCommand(() -> swerve.resetPose(path.getPreviewStartingHolonomicPose())),
    //   AutoBuilder.followPath(path),
    //   new InstantCommand(() -> swerve.stopModules())
    // );
    
    // (2d0): PRELOAD AUTO IF PROCESSING CAUSES DELAY
    //return new PathPlannerAuto("BlueAuto1(Mid)");
    return autoChooser.getSelected();
  }
}
