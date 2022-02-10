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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.PneumaticChannelConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.CANidConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  // Front & Back Climb Solenoids
  public DoubleSolenoid climbRight = null;
  public DoubleSolenoid climbLeft = null;

  public CANSparkMax climbRightMotor = null;
  public CANSparkMax climbLeftMotor = null;

  // ==============================================================
  // Identify encoders and PID controllers
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private SparkMaxPIDController climbPIDController;

  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
  private NetworkTableEntry cbLeftPos = climberTab.addPersistent("Left Position", 0.0).getEntry();
  private NetworkTableEntry cbRightPos = climberTab.addPersistent("Right Position", 0.0).getEntry();
  private NetworkTableEntry cbSetPoint = climberTab.addPersistent("PIDSetpoint", 0.0).getEntry();
  private NetworkTableEntry cbAtTarget = climberTab.addPersistent("At Target", false).getEntry();

  private double setPoint = 0;

  public Climber() {

    System.out.println("Climber Constructor Starting");

    // Initialize climb solenoids
    climbLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticChannelConstants.kClimbLeftExtend,
        PneumaticChannelConstants.kClimbLeftRetract);
    climbRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticChannelConstants.kClimbRightExtend,
        PneumaticChannelConstants.kClimbRightRetract);

    // ==============================================================
    // Identify left and right Motors

    climbLeftMotor = new CANSparkMax(CANidConstants.kClimbLeftMotor, MotorType.kBrushless);
    climbRightMotor = new CANSparkMax(CANidConstants.kClimbRightMotor, MotorType.kBrushless);

    climbLeftMotor.restoreFactoryDefaults();
    climbRightMotor.restoreFactoryDefaults();

    climbLeftMotor.setIdleMode(IdleMode.kBrake);
    climbRightMotor.setIdleMode(IdleMode.kBrake);

    // Group the left and right motors
    climbRightMotor.follow(climbLeftMotor, true); // invert direction of right motor

    // ==============================================================
    // Identify left and right Encoders

    leftEncoder = climbLeftMotor.getEncoder();
    rightEncoder = climbRightMotor.getEncoder();

    leftEncoder.setPositionConversionFactor(ClimberConstants.kPosFactorIPC);
    rightEncoder.setPositionConversionFactor(ClimberConstants.kPosFactorIPC);

    // ==============================================================
    // Identify PID controller
    climbPIDController = climbLeftMotor.getPIDController();

    climbPIDController.setP(ClimberConstants.kP);
    climbPIDController.setI(ClimberConstants.kI);
    climbPIDController.setD(ClimberConstants.kD);
    // climbPIDController.setIZone(ClimberConstants.kIz);
    // climbPIDController.setFF(ClimberConstants.kFF);
    // climbPIDController.setOutputRange(ClimberConstants.kMinOutput,
    // ClimberConstants.kMaxOutput);

    System.out.println("Climber Constructor Finished");
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
    // m_distPIDController.setSetpoint(distance * ChassisConstants.kPosFactor);
    this.setPoint = setPoint;
    cbSetPoint.setDouble(setPoint);
    climbPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }
}
