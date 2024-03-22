// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.SetIntakeLevel;
import frc.robot.commands.SetIntakeState;
import edu.wpi.first.math.controller.PIDController;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intakeRotationMotor;
  WPI_TalonSRX intakeRollerMotor;

  RelativeEncoder intakeRotationEncoder;

  PIDController intakePID;

  DigitalInput topLimitSwitch;
  Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

  private boolean isAtTarget;

  private IntakeLevels currentLevel;
  private IntakeStates currentState;

  public Intake() {

    intakeRotationMotor = new CANSparkMax(IntakeConstants.kIntakeRotationMotorID, MotorType.kBrushless);
    intakeRollerMotor = new WPI_TalonSRX(IntakeConstants.kIntakeRollerMotorID);
    topLimitSwitch = new DigitalInput(IntakeConstants.kTopLimitSwitchID);

    intakeRotationEncoder = intakeRotationMotor.getEncoder();
    intakeRotationMotor.setInverted(true);

    //intakeRotationMotor.setInverted(true);
    //intakeRollerMotor.setInverted(true);
  
    intakePID = new PIDController(IntakeConstants.kIntakeKP, IntakeConstants.kIntakeKI, IntakeConstants.kIntakeKD);
    intakePID.setTolerance(3.5); // encoder ticks

    distanceSensor.setAutomaticMode(true);
    distanceSensor.setEnabled(true);
    distanceSensor.setRangeProfile(RangeProfile.kHighAccuracy);

    currentLevel = IntakeLevels.STOWED; // change based on starting position
    currentState = IntakeStates.ZERO; // starts with 0 speed

    setBrakeMode();
  }

  public enum IntakeLevels {
    GROUND, // Ground pickup
    AMP, // Amp scoring
    STOWED, // Neutral position / feed to shooter
  }

  public enum IntakeStates {
    ZERO, // No speed
    INTAKE, // Intake note
    FEED, // Feed to shooter
    EJECT, // Amp scoring
    BOUNCE, // Bounce note before shooting
  }

  public double intakeStateToSpeed(IntakeStates desiredState) {
    switch(desiredState) {
      case ZERO:
        return 0.0;
      case INTAKE:
        return -0.35;
      case FEED:  
        return 0.90;
      case EJECT:
        return 0.373;
      case BOUNCE:
        return -0.30;
      default:
        return 0.0;
    }
  }

  public double intakeLevelToEncoderPosition(IntakeLevels desiredLevel) {
    switch(desiredLevel) {
      case GROUND: 
        return -114.42947387695312;
      case AMP:
        return -43.3817138861875;
      case STOWED:
        return 0.0;
      default:
        return 0.0;
    }
  }

  public IntakeLevels getCurrentLevel() {
    return this.currentLevel;
  }

  public IntakeStates getCurrentState() {
    return this.currentState;
  }

  public void setLevel(IntakeLevels desiredLevel) {
    this.currentLevel = desiredLevel;
  }

  public void setState(IntakeStates desiredState) {
    this.currentState = desiredState;
  }

  public boolean hasReachedMax() {
    return !topLimitSwitch.get();
  }

  public void resetEncoder(double ticks) {
    // set encoder position to constant at top limit
    intakeRotationEncoder.setPosition(ticks);
  }

  public boolean noteLoaded() {
    if(distanceSensor.getRange() < 15.5) { // set measurement
      return true;
    }
    else {
      return false;
    }
  }

  public void setRotationMotor(double speed) {
    intakeRotationMotor.set(speed);
  }

  public void stopRotationMotor() {
    intakeRotationMotor.stopMotor();
  }

  private void setRollerMotor(double speed) {
    if(speed == 0.0) {
      intakeRollerMotor.stopMotor();
    }
    else {
      intakeRollerMotor.set(speed);
    } 
  }

  private double safetyClamp(double speed) {
    if(hasReachedMax() && speed > 0) {
      return 0;
    }
    if(speed > IntakeConstants.kIntakeMaxSpeed) {
      return IntakeConstants.kIntakeMaxSpeed;
    }
    if(speed < -IntakeConstants.kIntakeMaxSpeed) {
      return -IntakeConstants.kIntakeMaxSpeed;
    }
    return speed;
  }

  public void setBrakeMode() {
    intakeRotationMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    intakeRotationMotor.setIdleMode(IdleMode.kCoast);
  }

  public Command intakeAndLoadCommand() {
    // return new SetIntakeLevel(IntakeLevels.GROUND)
    //   .andThen(
    //     new SetIntakeState(IntakeStates.INTAKE).until(() -> noteLoaded()))
    //   .andThen(
    //     new SetIntakeState(IntakeStates.ZERO))
    //   .andThen(new SetIntakeLevel(IntakeLevels.STOWED));

    return new SetIntakeLevel(IntakeLevels.GROUND)
      .andThen(
        new SetIntakeState(IntakeStates.INTAKE))
      .andThen(
        new WaitUntilCommand(this::noteLoaded))
      .andThen(
        new SetIntakeState(IntakeStates.ZERO))
      .andThen(new SetIntakeLevel(IntakeLevels.STOWED));
  }

  public Command autoIntakeAndLoadCommand() {
    return new SetIntakeLevel(IntakeLevels.GROUND)
      .andThen(
        new SetIntakeState(IntakeStates.INTAKE))
      .andThen(
        new WaitCommand(2.5))
      .andThen(
        new SetIntakeState(IntakeStates.ZERO))
      .andThen(new SetIntakeLevel(IntakeLevels.STOWED));
  }

  public Command getAmpShootCommand() {
    return new SetIntakeLevel(IntakeLevels.AMP)
      .andThen(new WaitCommand(1.5))
      .andThen(new SetIntakeState(IntakeStates.EJECT))
      .andThen(new WaitCommand(0.5))
      .andThen(new SetIntakeState(IntakeStates.ZERO))
      .andThen(new SetIntakeLevel(IntakeLevels.STOWED));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(intakeRotationEncoder.getPosition());
    //System.out.println("current level: " + this.currentLevel);
    //System.out.println(distanceSensor.getRange());

    double currentPosition = intakeRotationEncoder.getPosition();
    double targetPosition = intakeLevelToEncoderPosition(currentLevel);

    if(hasReachedMax()) {
      resetEncoder(0); 
    }

    if(currentState == IntakeStates.INTAKE && noteLoaded()) { // stop intake if note is loaded
      currentState = IntakeStates.ZERO;
    }

    setRollerMotor(intakeStateToSpeed(this.currentState));

    double calculatedSpeed = intakePID.calculate(currentPosition, targetPosition);
    double clampedSpeed = safetyClamp(calculatedSpeed);

    if(!intakePID.atSetpoint()) {
      intakeRotationMotor.set(clampedSpeed);
      isAtTarget = false;
      //System.out.println("changing level");
    }
    else {
      intakeRotationMotor.stopMotor();
      isAtTarget = true;
    }
  }
}
