// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private CANSparkMax shootLeadMotor;
  private CANSparkMax shootFollowMotor;

  private SparkMaxPIDController shootPIDController;
  private RelativeEncoder shootEncoder;

  public DoubleSolenoid plunger = null;

  private double shootSetPoint = 0.0;

  private Library lib = new Library();

  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  private NetworkTableEntry sbShootVel = shooterTab.addPersistent("ShootVelocity", 0).getEntry();
  private NetworkTableEntry sbShootSetPoint = shooterTab.addPersistent("Shoot SetPoint", 0).getEntry();

  public boolean running = false;

  public Shooter() {
    //Define motors
    shootLeadMotor = new CANSparkMax(CANidConstants.kShooterLeadMotor, MotorType.kBrushless);
    shootFollowMotor = new CANSparkMax(CANidConstants.kShooterFollowerMotor, MotorType.kBrushless);

    shootLeadMotor.restoreFactoryDefaults();
    shootFollowMotor.restoreFactoryDefaults();
    shootLeadMotor.clearFaults();
    shootFollowMotor.clearFaults();
    shootLeadMotor.setIdleMode(IdleMode.kBrake);
    shootFollowMotor.setIdleMode(IdleMode.kBrake);

    shootFollowMotor.follow(shootLeadMotor, true);

    //shootFollowMotor.setInverted(false);

    shootPIDController = shootLeadMotor.getPIDController();

    // Encoder object created to display position values
    shootEncoder = shootLeadMotor.getEncoder();

    shootPIDController.setP(ShooterConstants.kP);
    shootPIDController.setI(ShooterConstants.kI);
    shootPIDController.setD(ShooterConstants.kD);
    shootPIDController.setIZone(ShooterConstants.kIz);
    shootPIDController.setFF(ShooterConstants.kFF);
    shootPIDController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", ShooterConstants.kP);
    SmartDashboard.putNumber("I Gain", ShooterConstants.kI);
    SmartDashboard.putNumber("D Gain", ShooterConstants.kD);
    SmartDashboard.putNumber("I Zone", ShooterConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", ShooterConstants.kFF);
    SmartDashboard.putNumber("Max Output", ShooterConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ShooterConstants.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    plunger = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ShooterConstants.kPlungerExtend, ShooterConstants.kPlungerRetract);
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
 
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != ShooterConstants.kP)) { shootPIDController.setP(p); ShooterConstants.kP = p; }
    if((i != ShooterConstants.kI)) { shootPIDController.setI(i); ShooterConstants.kI = i; }
    if((d != ShooterConstants.kD)) { shootPIDController.setD(d); ShooterConstants.kD = d; }
    if((iz != ShooterConstants.kIz)) { shootPIDController.setIZone(iz); ShooterConstants.kIz = iz; }
    if((ff != ShooterConstants.kFF)) { shootPIDController.setFF(ff); ShooterConstants.kFF = ff; }
    if((max != ShooterConstants.kMaxOutput) || (min != ShooterConstants.kMinOutput)) { 
      shootPIDController.setOutputRange(min, max); 
      ShooterConstants.kMinOutput = min; ShooterConstants.kMaxOutput = max;  
    }

    sbShootVel.setDouble(getShootVelocity());
    sbShootSetPoint.setDouble(shootSetPoint);
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

  public void plungerExtend() {
    plunger.set(Value.kForward);
  }

  public void plungerRetract() {
    plunger.set(Value.kReverse);
  }
}
