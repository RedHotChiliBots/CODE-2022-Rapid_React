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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTableEntry;

import frc.robot.Library;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.PneumaticChannelConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  // ==============================================================
  // Define Motors
  private final CANSparkMax shootLeadMotor = new CANSparkMax(CANidConstants.kShooterLeadMotor, MotorType.kBrushless);
  private final CANSparkMax shootFollowMotor = new CANSparkMax(CANidConstants.kShooterFollowerMotor,
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

  public double getShootVelocity() {
    return shootEncoder.getVelocity();
  }

  public void setShootVelocity(double rpm) {
    this.shootSetPoint = lib.Clip(-rpm, ShooterConstants.kMaxRPM, ShooterConstants.kMinRPM);
    shootPIDController.setReference(shootSetPoint, ControlType.kVelocity);
  }

  public void stopShoot() {
    shootLeadMotor.set(0);
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
}
