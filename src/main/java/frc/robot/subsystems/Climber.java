// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.PneumaticChannelConstants;
import frc.robot.Constants.CANidConstants;


public class Climber extends SubsystemBase {

  // ==============================================================
  // Define Solenoids
  private final DoubleSolenoid climbRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticChannelConstants.kClimbRightExtend,
  PneumaticChannelConstants.kClimbRightRetract);
  private final DoubleSolenoid climbLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticChannelConstants.kClimbLeftExtend,
  PneumaticChannelConstants.kClimbLeftRetract);

  // ==============================================================
  // Define Motors
  private final CANSparkMax climbRightMotor = new CANSparkMax(CANidConstants.kClimbLeftMotor, MotorType.kBrushless);
  private final CANSparkMax climbLeftMotor = new CANSparkMax(CANidConstants.kClimbRightMotor, MotorType.kBrushless);

  // ==============================================================
  // Define encoders and PID controller
  private final RelativeEncoder leftEncoder = climbLeftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = climbRightMotor.getEncoder();
  private final SparkMaxPIDController climbPIDController = climbLeftMotor.getPIDController();

  // ==============================================================
  // Define Shuffleboard Tab
  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
  private final NetworkTableEntry cbLeftPos = climberTab.addPersistent("Left Position", 0.0).getEntry();
  private final NetworkTableEntry cbRightPos = climberTab.addPersistent("Right Position", 0.0).getEntry();
  private final NetworkTableEntry cbSetPoint = climberTab.addPersistent("PID Setpoint", 0.0).getEntry();
  private final NetworkTableEntry cbAtTarget = climberTab.addPersistent("At Target", false).getEntry();

  // ==============================================================
  // Define local variables
  private double setPoint = 0;


  public Climber() {

		System.out.println("+++++ Climber Constructor starting +++++");

    // ==============================================================
    // Configure left and right Motors
    climbLeftMotor.restoreFactoryDefaults();
    climbRightMotor.restoreFactoryDefaults();

    climbLeftMotor.setIdleMode(IdleMode.kBrake);
    climbRightMotor.setIdleMode(IdleMode.kBrake);

    // Group the left and right motors
    climbRightMotor.follow(climbLeftMotor, true); // invert direction of right motor

    // ==============================================================
    // Configure left and right Encoders
    leftEncoder.setPositionConversionFactor(ClimberConstants.kPosFactorIPC);
    rightEncoder.setPositionConversionFactor(ClimberConstants.kPosFactorIPC);

    // ==============================================================
    // Configure PID controller
    climbPIDController.setP(ClimberConstants.kP);
    climbPIDController.setI(ClimberConstants.kI);
    climbPIDController.setD(ClimberConstants.kD);
    climbPIDController.setIZone(ClimberConstants.kIz);
    climbPIDController.setFF(ClimberConstants.kFF);
    climbPIDController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

		System.out.println("----- Climber Constructor finished -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    cbLeftPos.setDouble(leftEncoder.getPosition());
    cbRightPos.setDouble(rightEncoder.getPosition());
    cbAtTarget.setBoolean(atTarget());
  }

  public void climberSwivel() {
    climbLeft.set(Value.kForward);
    climbRight.set(Value.kForward);
  }

  public void climberPerpendicular() {
    climbLeft.set(Value.kReverse);
    climbRight.set(Value.kReverse);
  }

  public boolean atTarget() {
    return Math.abs(setPoint - leftEncoder.getPosition()) <= ClimberConstants.kDistanceTolerance;
  }

  public void climbPosition(double setPoint) {
    this.setPoint = setPoint;
    cbSetPoint.setDouble(setPoint);
    climbPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }
}
