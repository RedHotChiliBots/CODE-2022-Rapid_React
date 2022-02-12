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
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.PneumaticChannelConstants;

public class Collector extends SubsystemBase {
  /** Creates a new Collector. */

  private CANSparkMax collectorMotor;

  private SparkMaxPIDController collectorPIDController;
  private RelativeEncoder collectorEncoder;

  private DoubleSolenoid collectorArm = null;

  private double collectorSetPoint = 0.0;
  
  private boolean running = false;

  private Library lib = new Library();

  private final ShuffleboardTab collectorTab = Shuffleboard.getTab("Collector");
  private NetworkTableEntry sbCollectVel = collectorTab.addPersistent("Collect Velocity", 0).getEntry();
  private NetworkTableEntry sbSetPoint = collectorTab.addPersistent("Collector SetPoint", 0).getEntry();
  private NetworkTableEntry sbAtTarget = collectorTab.addPersistent("At Target", false).getEntry();

  public Collector() {
    System.out.println("Shooter Constructor Starting");

    collectorMotor = new CANSparkMax(CANidConstants.kIntakeMotor, MotorType.kBrushless);
    
    collectorMotor.restoreFactoryDefaults();
    collectorMotor.clearFaults();
    collectorMotor.setIdleMode(IdleMode.kBrake);

    collectorPIDController = collectorMotor.getPIDController();

    // Encoder object created to display position values
    collectorEncoder = collectorMotor.getEncoder();

    collectorPIDController.setP(CollectorConstants.kP);
    collectorPIDController.setI(CollectorConstants.kI);
    collectorPIDController.setD(CollectorConstants.kD);
    collectorPIDController.setIZone(CollectorConstants.kIz);
    collectorPIDController.setFF(CollectorConstants.kFF);
    collectorPIDController.setOutputRange(CollectorConstants.kMinOutput, CollectorConstants.kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", CollectorConstants.kP);
    SmartDashboard.putNumber("I Gain", CollectorConstants.kI);
    SmartDashboard.putNumber("D Gain", CollectorConstants.kD);
    SmartDashboard.putNumber("I Zone", CollectorConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", CollectorConstants.kFF);
    SmartDashboard.putNumber("Max Output", CollectorConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", CollectorConstants.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    collectorArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticChannelConstants.kIntakeExtend,
        PneumaticChannelConstants.kIntakeRetract);

    System.out.println("Shooter Constructer Finished");
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

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != CollectorConstants.kP)) {
      collectorPIDController.setP(p);
      CollectorConstants.kP = p;
    }
    if ((i != CollectorConstants.kI)) {
      collectorPIDController.setI(i);
      CollectorConstants.kI = i;
    }
    if ((d != CollectorConstants.kD)) {
      collectorPIDController.setD(d);
      CollectorConstants.kD = d;
    }
    if ((iz != CollectorConstants.kIz)) {
      collectorPIDController.setIZone(iz);
      CollectorConstants.kIz = iz;
    }
    if ((ff != CollectorConstants.kFF)) {
      collectorPIDController.setFF(ff);
      CollectorConstants.kFF = ff;
    }
    if ((max != CollectorConstants.kMaxOutput) || (min != CollectorConstants.kMinOutput)) {
      collectorPIDController.setOutputRange(min, max);
      CollectorConstants.kMinOutput = min;
      CollectorConstants.kMaxOutput = max;
    }

    sbCollectVel.setDouble(getCollectorVelocity());
    sbSetPoint.setDouble(collectorSetPoint);
    sbAtTarget.setBoolean(atTarget());
  }

   public double getCollectorVelocity() {
    return collectorEncoder.getVelocity();
  }

  public void setCollectorVelocity(double rpm) {
    this.collectorSetPoint = lib.Clip(-rpm, CollectorConstants.kMaxRPM, CollectorConstants.kMinRPM);
    collectorPIDController.setReference(collectorSetPoint, ControlType.kVelocity);
  }

  public void stopCollect() {
    collectorMotor.set(0);
  }

  public boolean atTarget() {
    return Math.abs(collectorSetPoint - getCollectorVelocity()) <= CollectorConstants.kVelocityTolerance;
  }

  public void collectorArmExtend() {
    collectorArm.set(Value.kForward);
  }

  public void collectorArmRetract() {
    collectorArm.set(Value.kReverse);
  }

  public void setRunning(boolean r) {
    this.running = r;
  }

  public boolean isRunning() {
    return running;
  }
}
