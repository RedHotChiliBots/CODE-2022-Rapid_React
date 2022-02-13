// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTableEntry;

import frc.robot.Library;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.DIOChannelConstants;
import frc.robot.Constants.PWMChannelConstants;
import frc.robot.Constants.PneumaticChannelConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  // ==============================================================
  // Define Motors
  private final CANSparkMax shootLeadMotor = new CANSparkMax(
      CANidConstants.kShooterLeadMotor,
      MotorType.kBrushless);
  private final CANSparkMax shootFollowMotor = new CANSparkMax(
      CANidConstants.kShooterFollowerMotor,
      MotorType.kBrushless);

  // ==============================================================
  // Define PID Controller
  private final SparkMaxPIDController shootPIDController = shootLeadMotor.getPIDController();

  // ==============================================================
  // Define Encoder
  private final RelativeEncoder shootEncoder = shootLeadMotor.getEncoder();

  // ==============================================================
  // Define Solenoid
  private final DoubleSolenoid plunger = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      PneumaticChannelConstants.kPlungerExtend,
      PneumaticChannelConstants.kPlungerRetract);

  // ==============================================================
  // Define Digital Inputs
  private final DigitalInput shooterEntering = new DigitalInput(DIOChannelConstants.kShooterEntering);
  private final DigitalInput shooterExiting = new DigitalInput(DIOChannelConstants.kShooterExiting);

  // ==============================================================
  // Define PWM Connections (Servos)
  private final Servo leftServo = new Servo(PWMChannelConstants.kShooterLeftServo);
  private final Servo rightServo = new Servo(PWMChannelConstants.kShooterRightServo);

  // ==============================================================
  // Define Library
  private final Library lib = new Library();

  // ==============================================================
  // Define Suffleboard Tab
  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  private final NetworkTableEntry sbShootVel = shooterTab.addPersistent("ShootVelocity", 0).getEntry();
  private final NetworkTableEntry sbSetPoint = shooterTab.addPersistent("Shoot SetPoint", 0).getEntry();
  private final NetworkTableEntry sbAtTarget = shooterTab.addPersistent("At Target", false).getEntry();

  // ==============================================================
  // Define local variables
  private double shootSetPoint = 0.0;
  private boolean running = false;

  public enum ShooterState {
    NA,
    EMPTY,
    ENTERING,
    CONTROLLED
  }

  private ShooterState shooterState = ShooterState.NA;

  public enum GuardState {
    NA,
    OPEN,
    CLOSED
  }

  private GuardState guardState = GuardState.NA;

  public Shooter() {
    System.out.println("+++++ Shooter Constructor starting +++++");

    // ==============================================================
    // Configure Motors
    shootLeadMotor.restoreFactoryDefaults();
    shootFollowMotor.restoreFactoryDefaults();

    shootLeadMotor.clearFaults();
    shootFollowMotor.clearFaults();

    shootLeadMotor.setIdleMode(IdleMode.kBrake);
    shootFollowMotor.setIdleMode(IdleMode.kBrake);

    shootFollowMotor.follow(shootLeadMotor, true);

    // ==============================================================
    // Configure PID Controller
    shootPIDController.setP(ShooterConstants.kP);
    shootPIDController.setI(ShooterConstants.kI);
    shootPIDController.setD(ShooterConstants.kD);
    shootPIDController.setIZone(ShooterConstants.kIz);
    shootPIDController.setFF(ShooterConstants.kFF);
    shootPIDController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    // ==============================================================
    // Display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", ShooterConstants.kP);
    SmartDashboard.putNumber("I Gain", ShooterConstants.kI);
    SmartDashboard.putNumber("D Gain", ShooterConstants.kD);
    SmartDashboard.putNumber("I Zone", ShooterConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", ShooterConstants.kFF);
    SmartDashboard.putNumber("Max Output", ShooterConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ShooterConstants.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    // ==============================================================
    // Initialize devices before starting
    setShootVelocity(0.0);
    servoOpen();
    plungerRetract();

    System.out.println("----- Shooter Constructor finished -----");
  }

  @Override
  public void periodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed from PID setting, then
    // write new values to PID controller
    if (p != shootPIDController.getP()) {
      shootPIDController.setP(p);
    }
    if (i != shootPIDController.getI()) {
      shootPIDController.setI(i);
    }
    if (d != shootPIDController.getD()) {
      shootPIDController.setD(d);
    }
    if (iz != shootPIDController.getIZone()) {
      shootPIDController.setIZone(iz);
    }
    if (ff != shootPIDController.getFF()) {
      shootPIDController.setFF(ff);
    }
    if ((max != shootPIDController.getOutputMax()) || (min != shootPIDController.getOutputMin())) {
      shootPIDController.setOutputRange(min, max);
    }

    sbShootVel.setDouble(getShootVelocity());
    sbSetPoint.setDouble(shootSetPoint);
    sbAtTarget.setBoolean(atTarget());
  }

  public boolean isEntering() {
    return shooterEntering.get();
  }

  public boolean isExiting() {
    return shooterExiting.get();
  }

  public void setShooterState(ShooterState state) {
    shooterState = state;
  }

  public ShooterState getShooterState() {
    return shooterState;
  }

  public void setGuardState(GuardState state) {
    guardState = state;
  }

  public GuardState getGuardState() {
    return guardState;
  }

  public double getShootVelocity() {
    return shootEncoder.getVelocity();
  }

  public void setShootVelocity(double rpm) {
    this.shootSetPoint = lib.Clip(-rpm, ShooterConstants.kMaxRPM, ShooterConstants.kMinRPM);
    shootPIDController.setReference(shootSetPoint, ControlType.kVelocity);
  }

  public void stopShoot() {
    setShootVelocity(0.0);
  }

  public void setRunning(boolean r) {
    this.running = r;
  }

  public boolean isRunning() {
    return running;
  }

  public boolean atTarget() {
    return Math.abs(shootSetPoint - getShootVelocity()) <= ShooterConstants.kVelocityTolerance;
  }

  public void plungerExtend() {
    plunger.set(Value.kForward);
  }

  public void plungerRetract() {
    plunger.set(Value.kReverse);
  }

  public void servoOpen() {
    leftServo.setAngle(ShooterConstants.kServoLeftOpen);
    rightServo.setAngle(ShooterConstants.kServoRightOpen);
    // set servoState to OPEN in Command after 1 sec delay
  }

  public void servoClose() {
    leftServo.setAngle(ShooterConstants.kServoLeftClosed);
    rightServo.setAngle(ShooterConstants.kServoRightClosed);
    // set servoState to CLOSED in Command after 1 sec delay
  }
}
