// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The elevator subsystem uses PID to go to a given height. Unfortunately, in
 * it's current state PID
 * values for simulation are different than in the real world do to minor
 * differences.
 */
public class Shooter extends SubsystemBase {
  private CANSparkMax m_flywheel = new CANSparkMax(ShooterConstants.kFlywheelMotorPort, MotorType.kBrushless);
  private CANSparkMax m_triggerMotor = new CANSparkMax(ShooterConstants.kTriggerMotorPort, MotorType.kBrushless);
  private CANSparkMax m_flywheelLeft = new CANSparkMax(ShooterConstants.kFlywheelLeftMotorPort, MotorType.kBrushless);
  private CANSparkMax m_triggerMotorLeft = new CANSparkMax(ShooterConstants.kTriggerLeftMotorPort,
      MotorType.kBrushless);


  /**
   * Motors for the intake
   */
  private CANSparkMax m_upperIntake = new CANSparkMax(IntakeConstants.kLeftMotorPort, MotorType.kBrushless);
  private CANSparkMax m_lowerIntake = new CANSparkMax(IntakeConstants.kRightMotorPort, MotorType.kBrushless);
  /**
    * Adding a Playing with Fusion Time of Flight sensor. this is configured via a webserver running off the Rob0Rio http://10.88.02.2:5812
    * 
    */
  private TimeOfFlight intakeSensor = new TimeOfFlight(1);
  private TimeOfFlight shooterSensor = new TimeOfFlight(0);

  public Shooter() {
    m_flywheel.restoreFactoryDefaults();
    m_flywheel.setSmartCurrentLimit(80);
    m_flywheel.setIdleMode(IdleMode.kBrake);
    m_flywheel.setInverted(false);
    m_flywheelLeft.restoreFactoryDefaults();
    m_flywheelLeft.setSmartCurrentLimit(80);
    m_flywheelLeft.setIdleMode(IdleMode.kBrake);
    m_flywheelLeft.setInverted(true);
    m_triggerMotor.restoreFactoryDefaults();
    m_triggerMotor.setSmartCurrentLimit(80);
    m_triggerMotor.setIdleMode(IdleMode.kBrake);
    m_triggerMotor.setInverted(false);
    m_triggerMotorLeft.restoreFactoryDefaults();
    m_triggerMotorLeft.setSmartCurrentLimit(80);
    m_triggerMotorLeft.setIdleMode(IdleMode.kBrake);
    m_triggerMotorLeft.setInverted(true);

    m_upperIntake.restoreFactoryDefaults();
    m_upperIntake.setSmartCurrentLimit(20);
    m_upperIntake.setIdleMode(IdleMode.kCoast);
    m_upperIntake.setInverted(true);
    m_lowerIntake.restoreFactoryDefaults();
    m_lowerIntake.setSmartCurrentLimit(20);
    m_lowerIntake.setIdleMode(IdleMode.kCoast);
    m_lowerIntake.setInverted(true);
    intakeSensor.setRangingMode(RangingMode.Short, 24);
    shooterSensor.setRangingMode(RangingMode.Short, 24);
    //shooterSensor.setRangeOfInterest(-7,-2, -8,2);
    SmartDashboard.putNumber("IntakeSensor", intakeSensorDistance());
    SmartDashboard.putNumber("ShooterSensor", shooterSensorDistance());

    this.stop();
  }

  public void set(Double speed) {
    m_triggerMotor.set(speed);
    m_flywheel.set(speed);
  }

  public void stop() {
    m_triggerMotor.set(0);
    m_flywheel.set(0);
    m_triggerMotorLeft.set(0);
    m_flywheelLeft.set(0);
    m_lowerIntake.set(0);
    m_upperIntake.set(0);
  }

  public void startIntake() {
    m_triggerMotor.set(0.3);
    m_triggerMotorLeft.set(0.3);
    m_upperIntake.set(.75);
    m_lowerIntake.set(.75);
    m_flywheel.set(-.1);
    m_flywheelLeft.set(-.1);
  }

  public void intakeForAutonomous()
  {
    m_triggerMotor.set(0.2);
    m_triggerMotorLeft.set(0.2);
    m_upperIntake.setVoltage(0.75 * 12);
    m_lowerIntake.setVoltage(0.75 * 12);
    m_flywheel.set(-.5);
    m_flywheelLeft.set(-.5);
  }

  public boolean readyForShot() {
    return ((int)intakeSensor.getRange() < 300 && (int)shooterSensor.getRange() < 220);
  }

  public boolean intaking(){
    if( m_lowerIntake.get()>0 ) {
      return true;
    }
    return false;
  }

  

  public void bringNoteBackDown(){
    m_upperIntake.set(0);
    m_lowerIntake.set(0);
    m_triggerMotor.set(-0.1);
    m_triggerMotorLeft.set(-0.1);
    m_flywheel.set(.0);
    m_flywheelLeft.set(.0);
  }

  public void shoot() {
    m_flywheel.set(1);
    m_flywheelLeft.set(1);
    if (m_flywheel.getEncoder().getVelocity() > 5000){
      m_triggerMotor.set(.5);
      m_triggerMotorLeft.set(.5);
      m_upperIntake.set(0.3);
      m_lowerIntake.set(0.3);
    }
  }

  // version of shoot that doesn't use the intakes; note can't get stuck in them
  public void autoShoot()
  {
    m_triggerMotor.set(.5);
    m_triggerMotorLeft.set(.5);
    m_flywheel.set(1);
    m_flywheelLeft.set(1);
    m_upperIntake.set(0.1);
    m_lowerIntake.set(0.1);
  }

  public boolean hasNote(){
    if ((int)shooterSensor.getRange() < 175){
      return true;
    }
    return false;
  }

  public int intakeSensorDistance(){
    return (int)intakeSensor.getRange();
  }

   public int shooterSensorDistance(){
    return (int)shooterSensor.getRange();
  }


  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    // SmartDashboard.putData("Shooter Speed", m_pot);
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
    SmartDashboard.putNumber("IntakeSensor",intakeSensorDistance());
      SmartDashboard.putNumber("ShooterSensor",shooterSensorDistance());
      SmartDashboard.putBoolean("Has Note",hasNote());
      SmartDashboard.putData(this);
  }

  public void spitBackOut() {
    m_triggerMotor.set(-0.3);
    m_triggerMotorLeft.set(-0.3);
    m_upperIntake.set(-.4);
    m_lowerIntake.set(-.4);
    m_flywheel.set(-.1);
    m_flywheelLeft.set(-.1);
  }

public void startFlywheel() {
    m_flywheel.set(1);
    m_flywheelLeft.set(1);
}
}
