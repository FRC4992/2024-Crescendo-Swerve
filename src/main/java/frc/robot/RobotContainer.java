// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Extend;
import frc.robot.commands.IntakeFromShooter;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
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

  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public static NetworkTableEntry tx = table.getEntry("tx");
  public static NetworkTableEntry ta = table.getEntry("ta");
  public static NetworkTableEntry tv = table.getEntry("tv");

  public static double getLimelightX() {
    return tx.getDouble(0.0);
  }
  public static double getLimelightArea() {
    return ta.getDouble(0.0);
  }
  public static boolean getLimelightFoundTarget() {
    return tv.getDouble(0) == 1;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, m_driverController, false));
    //swerve.setDefaultCommand(new SwerveDriveCommand(swerve, m_driverController, 0.5, 0, 0, false));
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
    //m_driverController.b().onTrue(new ResetHeading());

    //m_driverController.y().onTrue(new Shoot());
    //m_driverController.a().whileTrue(new IntakeFromShooter());
    m_driverController.leftBumper().onTrue(new IntakeFromShooter());
    m_driverController.rightBumper().onTrue(new Shoot());
    // m_driverController.leftBumper().onTrue(new Extend());
    // m_driverController.rightBumper().onTrue(new Retract());

    m_driverController.y().whileTrue(new AutoAlign());

    m_driverController.b().onTrue(new SetIntakeLevel(IntakeLevels.GROUND));
    m_driverController.x().onTrue(new SetIntakeLevel(IntakeLevels.STOWED));
    m_driverController.a().onTrue(new SetIntakeLevel(IntakeLevels.AMP));
    m_driverController.povDown().whileTrue(new SetIntakeState(IntakeStates.INTAKE));
    m_driverController.povUp().whileTrue(new SetIntakeState(IntakeStates.FEED));
    m_driverController.povRight().whileTrue(new SetIntakeState(IntakeStates.EJECT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
