// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnalogIOConstants;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.ChassisConstants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


public class Chassis extends SubsystemBase {
  
	// ==============================================================
	// Define the left side motors, master and follower
	private CANSparkMax leftMaster;
	private CANSparkMax leftFollower;

	// Define the right side motors, master and follower
	private CANSparkMax rightMaster;
	private CANSparkMax rightFollower;

	private DifferentialDrive m_diffDrive;

	// ==============================================================
	// Identify encoders and PID controllers
  	private RelativeEncoder leftEncoder;
  	private RelativeEncoder rightEncoder;
	
	private SparkMaxPIDController leftPIDController;
	private SparkMaxPIDController rightPIDController;

  	private double setPoint = 0;

	// ==============================================================
	// Define autonomous support functions
	public DifferentialDriveKinematics m_kinematics;

	public DifferentialDriveOdometry m_odometry;

	// Create a voltage constraint to ensure we don't accelerate too fast
	private DifferentialDriveVoltageConstraint autoVoltageConstraint;

	// Create config for trajectory
	private TrajectoryConfig config;
	private TrajectoryConfig configReversed;

	// An example trajectory to follow. All units in meters.
	public Trajectory lineToRendezvousTrajectory;

	// ==============================================================
	// Initialize NavX AHRS board
	// Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
	private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

	// ==============================================================
	// Identify PDP and PCM
	private final PowerDistribution pdp = new PowerDistribution();
	private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

	// Identify compressor hi and lo sensors
	private final AnalogInput hiPressureSensor = new AnalogInput(AnalogIOConstants.kHiPressureChannel);
	private final AnalogInput loPressureSensor = new AnalogInput(AnalogIOConstants.kLoPressureChannel);

	// ==============================================================
	// Define Shuffleboard data
	private final ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
	private NetworkTableEntry sbRobotAngle = chassisTab.addPersistent("Robot Angle", 0).getEntry();
	private NetworkTableEntry sbLeftPos = chassisTab.addPersistent("ML Position", 0).getEntry();
	private NetworkTableEntry sbLeftVel = chassisTab.addPersistent("ML Velocity", 0).getEntry();
	private NetworkTableEntry sbRightPos = chassisTab.addPersistent("MR Position", 0).getEntry();
	private NetworkTableEntry sbRightVel = chassisTab.addPersistent("MR Velocity", 0).getEntry();
	private NetworkTableEntry sbLeftPow = chassisTab.addPersistent("ML Power", 0).getEntry();
	private NetworkTableEntry sbRightPow = chassisTab.addPersistent("MR Power", 0).getEntry();

	private final ShuffleboardTab pneumaticsTab = Shuffleboard.getTab("Pneumatics");
	private NetworkTableEntry sbHiPressure = pneumaticsTab.addPersistent("Hi Pressure", 0).getEntry();
	private NetworkTableEntry sbLoPressure = pneumaticsTab.addPersistent("Lo Pressure", 0).getEntry();

  	public Chassis() {
		System.out.println("+++++ Chassis Constructor starting ...");

		pdp.clearStickyFaults();

		// ==============================================================
		// Define the left side motors, master and follower
		leftMaster = new CANSparkMax(CANidConstants.kLeftMasterMotor, MotorType.kBrushless);
		leftFollower = new CANSparkMax(CANidConstants.kLeftFollowerMotor, MotorType.kBrushless);

		leftMaster.restoreFactoryDefaults();
    	leftFollower.restoreFactoryDefaults();

    	leftMaster.setIdleMode(IdleMode.kBrake);
    	leftFollower.setIdleMode(IdleMode.kBrake);

		// Define the right side motors, master and follower
		rightMaster = new CANSparkMax(CANidConstants.kRightMasterMotor, MotorType.kBrushless);
		rightFollower = new CANSparkMax(CANidConstants.kRightFollowerMotor, MotorType.kBrushless);

		rightMaster.restoreFactoryDefaults();
    	rightFollower.restoreFactoryDefaults();

		rightMaster.setIdleMode(IdleMode.kBrake);
    	rightFollower.setIdleMode(IdleMode.kBrake);

		rightMaster.setInverted(true);
		rightFollower.setInverted(true);
	
    	// Group the left and right motors
		leftFollower.follow(leftMaster);
		rightFollower.follow(rightMaster);

		m_diffDrive = new DifferentialDrive(leftMaster, rightMaster);

    	// ==============================================================
		// Identify PID controller
		leftPIDController = leftMaster.getPIDController();
		rightPIDController = rightMaster.getPIDController();
		
    	leftPIDController.setP(ChassisConstants.kP);
    	leftPIDController.setI(ChassisConstants.kI);
    	leftPIDController.setD(ChassisConstants.kD);
    	// leftPIDController.setIZone(ClimberConstants.kIz);
    	// leftPIDController.setFF(ClimberConstants.kFF);
    	// leftPIDController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);
		
    	rightPIDController.setP(ChassisConstants.kP);
    	rightPIDController.setI(ChassisConstants.kI);
    	rightPIDController.setD(ChassisConstants.kD);
    	// rightPIDController.setIZone(ClimberConstants.kIz);
    	// rightPIDController.setFF(ClimberConstants.kFF);
    	// rightPIDController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

		// ==============================================================
		// Identify encoders and PID controllers
		leftEncoder = leftMaster.getEncoder();
		rightEncoder = rightMaster.getEncoder();

		leftEncoder.setPositionConversionFactor(ChassisConstants.kPosFactorIPC);
		rightEncoder.setPositionConversionFactor(ChassisConstants.kPosFactorIPC);

		// ==============================================================
		// Define autonomous support functions
		m_kinematics = new DifferentialDriveKinematics(ChassisConstants.kTrackWidth);

		m_odometry = new DifferentialDriveOdometry(getAngle());

		// Create a voltage constraint to ensure we don't accelerate too fast
		autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(ChassisConstants.ksVolts, ChassisConstants.kvVoltSecondsPerMeter, ChassisConstants.kaVoltSecondsSquaredPerMeter), m_kinematics, 10);

		// Create config for trajectory
		config = new TrajectoryConfig(ChassisConstants.kMaxSpeedMetersPerSecond,
			ChassisConstants.kMaxAccelerationMetersPerSecondSquared)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(m_kinematics)
			// Apply the voltage constraint
			.addConstraint(autoVoltageConstraint)
			.setReversed(false);
						
		
		configReversed = new TrajectoryConfig(ChassisConstants.kMaxSpeedMetersPerSecond,
			ChassisConstants.kMaxAccelerationMetersPerSecondSquared)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(m_kinematics)
			// Apply the voltage constraint
			.addConstraint(autoVoltageConstraint)
			.setReversed(true);

		lineToRendezvousTrajectory = TrajectoryGenerator.generateTrajectory(
			new Pose2d(0, 1.7, new Rotation2d(0)),
			List.of(new Translation2d(1, 1)),
			new Pose2d(2.9, 3.9, new Rotation2d(0)),
			// Pass config
			config);

		leftMaster.getEncoder().setPositionConversionFactor(ChassisConstants.kPosFactor);
		rightMaster.getEncoder().setPositionConversionFactor(ChassisConstants.kPosFactor);
	
		leftMaster.getEncoder().setVelocityConversionFactor(ChassisConstants.kVelFactor);
		rightMaster.getEncoder().setVelocityConversionFactor(ChassisConstants.kVelFactor);
	
		chassisTab.addPersistent("ML Pos Factor", leftMaster.getEncoder().getPositionConversionFactor());
		chassisTab.addPersistent("MR Pos Factor", rightMaster.getEncoder().getPositionConversionFactor());
		chassisTab.addPersistent("ML Vel Factor", leftMaster.getEncoder().getVelocityConversionFactor());
		chassisTab.addPersistent("MR Vel Factor", rightMaster.getEncoder().getVelocityConversionFactor());
			
		// Reset the current encoder positions to zero
		leftMaster.getEncoder().setPosition(0.0);
		rightMaster.getEncoder().setPosition(0.0);
	
		resetFieldPosition(0.0, 0.0);
	
		System.out.println("----- Chassis Constructor finished ...");
  	}

  	@Override
  	public void periodic() {
		sbRobotAngle.setDouble(getAngle().getDegrees());
		sbLeftPos.setDouble(leftMaster.getEncoder().getPosition());
		sbLeftVel.setDouble(leftMaster.getEncoder().getVelocity());
		sbRightPos.setDouble(rightMaster.getEncoder().getPosition());
		sbRightVel.setDouble(rightMaster.getEncoder().getVelocity());
		sbLeftPow.setDouble(leftMaster.get());
		sbRightPow.setDouble(rightMaster.get());

		sbHiPressure.setDouble(getHiPressure());
		sbLoPressure.setDouble(getLoPressure());

		// Update field position - for autonomous
		updateOdometry();

		Pose2d pose = m_odometry.getPoseMeters();
		Translation2d trans = pose.getTranslation();
		Rotation2d rot = pose.getRotation();
  	}

  	public DifferentialDriveOdometry getOdometry() {
		return m_odometry;
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public DifferentialDriveKinematics getKinematics() {
		return m_kinematics;
	}

	public SparkMaxPIDController getLeftPID() {
		return leftPIDController;
	}

	public SparkMaxPIDController getRightPID() {
		return rightPIDController;
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(leftMaster.getEncoder().getVelocity(), rightMaster.getEncoder().getVelocity());
	}

	public double getDuration(Trajectory t) {
		return t.getTotalTimeSeconds();
	}

	public double getPitch() {
		return ahrs.getPitch();
	}

	public void driveTankVolts(double left, double right) {
		m_diffDrive.tankDrive(left, right);
	}

	public void driveTank(double left, double right) {
		m_diffDrive.tankDrive(-left, -right);
	}

	public void driveArcade(double spd, double rot) {
		m_diffDrive.arcadeDrive(-spd, rot);
	}

	public void resetFieldPosition(double x, double y) {
		ahrs.zeroYaw();
		leftMaster.getEncoder().setPosition(0.0);
		rightMaster.getEncoder().setPosition(0.0);
		m_odometry.resetPosition(new Pose2d(x, y, getAngle()), getAngle());
	}

	/**
	 * Returns the angle of the robot as a Rotation2d.
	 *
	 * @return The angle of the robot.
	 */
	public Rotation2d getAngle() {
		// Negating the angle because WPILib gyros are CW positive.
		return Rotation2d.fromDegrees(-ahrs.getYaw());
	}

	/**
	 * Sets the desired wheel speeds.
	 *
	 * @param speeds The desired wheel speeds.
	 */
	// public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
	// 	double leftFeedforward = 0.0;// m_Feedforward.calculate(speeds.leftMetersPerSecond);
	// 	double rightFeedforward = 0.0; // m_Feedforward.calculate(speeds.rightMetersPerSecond);

	// 	double leftOutput = leftPIDController.calculate(leftMaster.getEncoder().getVelocity(), speeds.leftMetersPerSecond);
	// 	double rightOutput = rightPIDController.calculate(rightMaster.getEncoder().getVelocity(), speeds.rightMetersPerSecond);

	// 	leftMaster.set(leftOutput + leftFeedforward);
	// 	rightMaster.set(rightOutput + rightFeedforward);
	// }

	/**
	 * Drives the robot with the given linear velocity and angular velocity.
	 *
	 * @param xSpeed Linear velocity in m/s.
	 * @param rot    Angular velocity in rad/s.
	 */
	// @SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double xRot) {
		var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, -xRot));
		leftMaster.set(wheelSpeeds.leftMetersPerSecond);
	 	rightMaster.set(wheelSpeeds.rightMetersPerSecond);
	}

	/**
	 * Updates the field-relative position.
	 */

	public void updateOdometry() {
		// SDS 2/12/29 - testing with inverted group rather than inverting encoder here
		// m_odometry.update(getAngle(), m_leftEncoder.getPosition(),
		// -m_rightEncoder.getPosition());
		m_odometry.update(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());
	}

	public void driveTrajectory(double left, double right) {
		leftMaster.set(left);
		rightMaster.set(right);
	}

	public void drivePosition(double setPoint) {
		// m_distPIDController.setSetpoint(distance * ChassisConstants.kPosFactor);
    	this.setPoint = setPoint;
		SmartDashboard.putNumber("PIDSetpoint",  setPoint);
		leftPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
		rightPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
	}

	public boolean atTarget(){
    	return Math.abs(setPoint - leftEncoder.getPosition()) <= ChassisConstants.kDistanceTolerance &&
		Math.abs(setPoint - rightEncoder.getPosition()) <= ChassisConstants.kDistanceTolerance;
  	}

	public double getLoPressure() {
		return 250.0 * (loPressureSensor.getVoltage() / AnalogIOConstants.kInputVoltage) - 25.0;
	}

	public double getHiPressure() {
		return 250.0 * (hiPressureSensor.getVoltage() / AnalogIOConstants.kInputVoltage) - 25.0;
	}

}
