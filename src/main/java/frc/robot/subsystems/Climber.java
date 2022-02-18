// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.PDPChannelConstants;
import frc.robot.Constants.PneumaticChannelConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANidConstants;

public class Climber extends SubsystemBase {

	// ==============================================================
	// Define Solenoids
	private final DoubleSolenoid climbRight = new DoubleSolenoid(
			PneumaticsModuleType.CTREPCM,
			PneumaticChannelConstants.kClimbRightExtend,
			PneumaticChannelConstants.kClimbRightRetract);
	private final DoubleSolenoid climbLeft = new DoubleSolenoid(
			PneumaticsModuleType.CTREPCM,
			PneumaticChannelConstants.kClimbLeftExtend,
			PneumaticChannelConstants.kClimbLeftRetract);

	private final DoubleSolenoid climbLatch = new DoubleSolenoid(
			PneumaticsModuleType.CTREPCM,
			PneumaticChannelConstants.kLatchOpen,
			PneumaticChannelConstants.kLatchClose);

	// ==============================================================
	// Define Motors
	private final CANSparkMax climbRightMotor = new CANSparkMax(
			CANidConstants.kClimbLeftMotor,
			MotorType.kBrushless);
	private final CANSparkMax climbLeftMotor = new CANSparkMax(
			CANidConstants.kClimbRightMotor,
			MotorType.kBrushless);

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
	private final NetworkTableEntry cbSwivel = climberTab.addPersistent("Swivel State", "").getEntry();
	private final NetworkTableEntry cbLeftPower = climberTab.addPersistent("Left Power", 0.0).getEntry();
	private final NetworkTableEntry cbRightPower = climberTab.addPersistent("Right Power", 0.0).getEntry();
	private final NetworkTableEntry cbLeftAmps = climberTab.addPersistent("Left Amps", 0.0).getEntry();
	private final NetworkTableEntry cbRightAmps = climberTab.addPersistent("Right Amps", 0.0).getEntry();

	// ==============================================================
	// Define local variables
	private double setPoint = 0;
	private Chassis chassis = null;
	private PowerDistribution pdp = null;
	Timer latchTimer = new Timer();
	Timer swivelTimer = new Timer();
	Timer initTimer = new Timer();

	public enum SwivelState {
		NA,
		PERPENDICULAR,
		SWIVEL
	}

	private SwivelState swivelState = SwivelState.NA;

	public enum LatchState {
		NA,
		OPEN,
		CLOSE
	}

	private LatchState latchState = LatchState.NA;

	public Climber(Chassis chassis) {

		System.out.println("+++++ Climber Constructor starting +++++");

		this.chassis = chassis;

		// ==============================================================
		// Configure left and right Motors
		climbLeftMotor.restoreFactoryDefaults();
		climbRightMotor.restoreFactoryDefaults();

		climbLeftMotor.clearFaults();
		climbRightMotor.clearFaults();

		climbLeftMotor.setIdleMode(IdleMode.kBrake);
		climbRightMotor.setIdleMode(IdleMode.kBrake);

		climbLeftMotor.setInverted(true);

		// // Group the left and right motors
		// climbRightMotor.follow(climbLeftMotor, true); // invert direction of right
		// motor

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

		// ==============================================================
		// Initialize devices before starting
		// climberInit();
		pdp = chassis.getPDP();

		climberPerpendicular();
		latchOpen();
		latchTimer.start();
		latchTimer.reset();
		swivelTimer.start();
		swivelTimer.reset();
		initTimer.start();
		initTimer.reset();

		System.out.println("----- Climber Constructor finished -----");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		cbLeftPos.setDouble(leftEncoder.getPosition());
		cbRightPos.setDouble(rightEncoder.getPosition());
		cbSetPoint.setDouble(setPoint);
		cbAtTarget.setBoolean(atTarget());
		cbSwivel.setString(swivelState.toString());
		cbLeftPower.setDouble(climbLeftMotor.get());
		cbRightPower.setDouble(climbRightMotor.get());
		cbLeftAmps.setDouble(pdp.getCurrent(PDPChannelConstants.kClimberLeft));
		cbRightAmps.setDouble(pdp.getCurrent(PDPChannelConstants.kClimberRight));
	}

	public SwivelState getSwivelState() {
		return swivelState;
	}

	public LatchState getLatchState() {
		return latchState;
	}

	public boolean atTarget() {
		return Math.abs(setPoint - leftEncoder.getPosition()) <= ClimberConstants.kDistanceTolerance;
	}

	public void climbPosition(double setPoint) {
		this.setPoint = setPoint;
		cbSetPoint.setDouble(setPoint);
		climbPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
	}

	public void climberInit() {
		climberInit(false);
	}

	public void climberInit(boolean noWait) {
		Thread thread = new Thread("ClimberInit") {
			@Override
			public void run() {
				boolean complete = false;
				boolean timedOut = false;
				boolean leftDone = false;
				boolean rightDone = false;
				double time = 0.0;
				initTimer.reset();

				DecimalFormat df = new DecimalFormat("#0.0000");
				df.setRoundingMode(RoundingMode.CEILING);

				System.out.println("climberInit pending start");

				climbLeftMotor.set(ClimberConstants.kInitSpeed);
				climbRightMotor.set(ClimberConstants.kInitSpeed);

				while (!complete) {
					time = initTimer.get();
					System.out.println(df.format(time) +
							" Left:  Amps: " + df.format(pdp.getCurrent(PDPChannelConstants.kClimberLeft)) + " Power: "
							+ df.format(climbLeftMotor.get()));
					System.out.println(df.format(time) +
							" Right: Amps: " + df.format(pdp.getCurrent(PDPChannelConstants.kClimberRight)) + " Power: "
							+ df.format(climbRightMotor.get()));

					if (initTimer.hasElapsed(ClimberConstants.kInitDelay) || noWait) {

						time = initTimer.get();
						System.out.println(df.format(time) + " climberInit starting");

						time = initTimer.get();
						System.out.println(df.format(time) +
								" Left:  Amps: " + df.format(pdp.getCurrent(PDPChannelConstants.kClimberLeft))
								+ " Power: " + df.format(climbLeftMotor.get()));
						System.out.println(df.format(time) +
								" Right: Amps: " + df.format(pdp.getCurrent(PDPChannelConstants.kClimberRight))
								+ " Power: " + df.format(climbRightMotor.get()));

						while (!(leftDone && rightDone) && !timedOut) {

							if (climbLeftMotor.getOutputCurrent() > ClimberConstants.kMaxAmps) {
								climbLeftMotor.set(0.0);
								rightEncoder.setPosition(0.0);
								leftDone = true;

								time = initTimer.get();
								System.out.println(df.format(time) + " climberInit left done");
							}
							if (climbRightMotor.getOutputCurrent() > ClimberConstants.kMaxAmps) {
								climbRightMotor.set(0.0);
								leftEncoder.setPosition(0.0);
								rightDone = true;

								time = initTimer.get();
								System.out.println(df.format(time) + " climberInit right done");
							}

							if (initTimer.hasElapsed(ClimberConstants.kInitSafety)) {
								timedOut = true;
								climbLeftMotor.set(0.0);
								climbRightMotor.set(0.0);

								time = initTimer.get();
								System.out.println(df.format(time) + " climberInit safety abort");
							}
							yield();
						}
						complete = true;
					} else {
						yield();
					}
				}

				if (!timedOut) {
					time = initTimer.get();
					System.out.println(df.format(time) + " climberInit configure motors");
					// Group the left and right motors
					climbRightMotor.follow(climbLeftMotor, true); // invert direction of right motor

					setPoint = 0.0;
					climbPosition(setPoint);
				}
				time = initTimer.get();
				System.out.println(df.format(time) + " climberInit finished");
			}
		};
		thread.start();
	}

	public void latchOpen() {
		latchOpen(false);
	}

	public void latchOpen(boolean noWait) {
		Thread thread = new Thread("LatchOpen") {
			@Override
			public void run() {
				boolean waiting = false;
				boolean complete = false;

				while (!complete) {
					if (!waiting) {
						climbLatch.set(Value.kForward);

						latchTimer.reset();
						waiting = true;

					} else {
						if (latchTimer.hasElapsed(ClimberConstants.kLatchDelay) || noWait) {

							latchState = LatchState.OPEN;
							waiting = false;
							complete = true;
						}
					}
				}
			}
		};
		thread.start();
	}

	public void latchClose() {
		latchClose(false);
	}

	public void latchClose(boolean noWait) {
		Thread thread = new Thread("LatchClose") {
			@Override
			public void run() {
				boolean waiting = false;
				boolean complete = false;

				while (!complete) {
					if (!waiting) {
						climbLatch.set(Value.kReverse);

						latchTimer.reset();
						waiting = true;

					} else {
						if (latchTimer.hasElapsed(ClimberConstants.kLatchDelay) || noWait) {

							latchState = LatchState.CLOSE;
							waiting = false;
							complete = true;
						}
					}
				}
			}
		};
		thread.start();
	}

	public void climberSwivel() {
		climberSwivel(false);
	}

	public void climberSwivel(boolean noWait) {
		Thread thread = new Thread("ClimbExtend") {
			@Override
			public void run() {
				boolean waiting = false;
				boolean complete = false;

				while (!complete) {
					if (!waiting) {
						climbLeft.set(Value.kForward);
						climbRight.set(Value.kForward);

						swivelTimer.reset();
						waiting = true;

					} else {
						if (swivelTimer.hasElapsed(ClimberConstants.kSwivelDelay) || noWait) {

							swivelState = SwivelState.SWIVEL;
							waiting = false;
							complete = true;
						}
					}
				}
			}
		};
		thread.start();
	}

	public void climberPerpendicular() {
		climberPerpendicular(false);
	}

	public void climberPerpendicular(boolean noWait) {
		Thread thread = new Thread("ClimberRetract") {
			@Override
			public void run() {
				boolean waiting = false;
				boolean complete = false;

				while (!complete) {
					if (!waiting) {
						climbLeft.set(Value.kReverse);
						climbRight.set(Value.kReverse);

						swivelTimer.reset();
						waiting = true;

					} else {
						if (swivelTimer.hasElapsed(ClimberConstants.kSwivelDelay) || noWait) {

							swivelState = SwivelState.PERPENDICULAR;
							waiting = false;
							complete = true;
						}
					}
				}
			}
		};
		thread.start();
	}
}
