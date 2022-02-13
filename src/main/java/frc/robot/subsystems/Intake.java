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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Library;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.DIOChannelConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PneumaticChannelConstants;

public class Intake extends SubsystemBase {

  // ==============================================================
  // Define Motor
  private final CANSparkMax intakeMotor = new CANSparkMax(
      CANidConstants.kIntakeMotor,
      MotorType.kBrushless);

  // ==============================================================
  // Define PID Controller
  private final SparkMaxPIDController intakePIDController = intakeMotor.getPIDController();

  // ==============================================================
  // Define Encoder
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  // ==============================================================
  // Define Solenoid
  private final DoubleSolenoid intakeArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      PneumaticChannelConstants.kIntakeExtend,
      PneumaticChannelConstants.kIntakeRetract);

  // ==============================================================
  // Define Digital Inputs
  private final DigitalInput intakeEntering = new DigitalInput(DIOChannelConstants.kIntakeEntering);

  // ==============================================================
  // Define Library
  private final Library lib = new Library();

  // ==============================================================
  // Define Shuffleboard Tab
  private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  private final NetworkTableEntry sbIntakeVel = intakeTab.addPersistent("Intake Velocity", 0).getEntry();
  private final NetworkTableEntry sbSetPoint = intakeTab.addPersistent("Intake SetPoint", 0).getEntry();
  private final NetworkTableEntry sbAtTarget = intakeTab.addPersistent("At Target", false).getEntry();

  // ==============================================================
  // Define Local Variables
  private double intakeSetPoint = 0.0;
  private boolean running = false;

  public enum IntakeState {
    NA,
    EMPTY,
    ENTERING
  }

  private IntakeState intakeState = IntakeState.NA;

  public Intake() {
    System.out.println("+++++ Intake Constructor starting +++++");

    // ==============================================================
    // Configure Motor
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.clearFaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);

    // ==============================================================
    // Configure PID Controller
    intakePIDController.setP(IntakeConstants.kP);
    intakePIDController.setI(IntakeConstants.kI);
    intakePIDController.setD(IntakeConstants.kD);
    intakePIDController.setIZone(IntakeConstants.kIz);
    intakePIDController.setFF(IntakeConstants.kFF);
    intakePIDController.setOutputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);

    // ==============================================================
    // Update Suffleboard Tab with static data
    SmartDashboard.putNumber("P Gain", IntakeConstants.kP);
    SmartDashboard.putNumber("I Gain", IntakeConstants.kI);
    SmartDashboard.putNumber("D Gain", IntakeConstants.kD);
    SmartDashboard.putNumber("I Zone", IntakeConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", IntakeConstants.kFF);
    SmartDashboard.putNumber("Max Output", IntakeConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", IntakeConstants.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    // ==============================================================
    // Initialize devices before starting
    setIntakeVelocity(0.0);
    intakeArmRetract();

    System.out.println("----- Intake Constructor finished -----");
  }

  @Override
  public void periodic() {
    // ==============================================================
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // ==============================================================
    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if (p != intakePIDController.getP()) {
      intakePIDController.setP(p);
    }
    if (i != intakePIDController.getI()) {
      intakePIDController.setI(i);
    }
    if (d != intakePIDController.getD()) {
      intakePIDController.setD(d);
    }
    if (iz != intakePIDController.getIZone()) {
      intakePIDController.setIZone(iz);
    }
    if (ff != intakePIDController.getFF()) {
      intakePIDController.setFF(ff);
    }
    if ((max != intakePIDController.getOutputMax()) || (min != intakePIDController.getOutputMin())) {
      intakePIDController.setOutputRange(min, max);
    }

    // ==============================================================
    // Update Shuffleboard Tab with dynamic data
    sbIntakeVel.setDouble(getIntakeVelocity());
    sbSetPoint.setDouble(intakeSetPoint);
    sbAtTarget.setBoolean(atTarget());
  }

  public boolean isEntering() {
    return intakeEntering.get();
  }

  public void setIntakeState(IntakeState state) {
    intakeState = state;
  }

  public IntakeState getState() {
    return intakeState;
  }

  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  public void setIntakeVelocity(double rpm) {
    this.intakeSetPoint = lib.Clip(-rpm, IntakeConstants.kMaxRPM, IntakeConstants.kMinRPM);
    intakePIDController.setReference(intakeSetPoint, ControlType.kVelocity);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public boolean atTarget() {
    return Math.abs(intakeSetPoint - getIntakeVelocity()) <= IntakeConstants.kVelocityTolerance;
  }

  public void intakeArmExtend() {
    intakeArm.set(Value.kForward);
  }

  public void intakeArmRetract() {
    intakeArm.set(Value.kReverse);
  }

  public void setRunning(boolean r) {
    this.running = r;
  }

  public boolean isRunning() {
    return running;
  }
}
