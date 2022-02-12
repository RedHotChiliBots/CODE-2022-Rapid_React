// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class CANidConstants {
		public static final int kPDP = 0;
		public static final int kCompressor = 0;

		public static final int kRightMasterMotor = 10;
		public static final int kRightFollowerMotor = 11;
		public static final int kLeftMasterMotor = 12;
		public static final int kLeftFollowerMotor = 13;

		public static final int kClimbRightMotor = 20;
		public static final int kClimbLeftMotor = 21;

		public static final int kShooterLeadMotor = 30;
		public static final int kShooterFollowerMotor = 31;

		public static final int kIntakeMotor = 40;

		public static final int kHopperMotor = 50;
	}

	public static final class PneumaticChannelConstants {
		public static final int kClimbLeftExtend = 0;
		public static final int kClimbLeftRetract = 1;
		public static final int kClimbRightExtend = 2;
		public static final int kClimbRightRetract = 3;

		public static final int kPlungerExtend = 4;
		public static final int kPlungerRetract = 5;

		public static final int kIntakeExtend = 6;
		public static final int kIntakeRetract = 7;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
	}

	public static final class AnalogIOConstants {
		public static final int kHiPressureChannel = 0;
		public static final int kLoPressureChannel = 1;

		public static final double kInputVoltage = 5.0;
	}

	public static final class ChassisConstants {
		// Constants for Drive PIDs
		public static final double kP = 0.5;
		public static final double kI = 0.0005;
		public static final double kD = 0.0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = -1.0;
		public static final double kMaxOutput = 1.0;

		public static final double kTrackWidth = Units.inchesToMeters(26.341); // meters
		public static final double kWheelCirc = Units.inchesToMeters(Math.PI * 8.0); // meters
		public static final int kEncoderResolution = 42; // not used, NEO's native units are rotations
		public static final double kGearBoxRatio = 10.71;
		public static final double kPosFactor = kWheelCirc / kGearBoxRatio; // Meters per Revolution
		public static final double kVelFactor = kWheelCirc / kGearBoxRatio / 60.0; // Meters per Second
		public static final double kCountsPerRevGearbox = kEncoderResolution * kGearBoxRatio;

		public static final double kPosFactorIPC = kWheelCirc / kCountsPerRevGearbox; // Meters per Revolution
		public static final double kPosFactorCPI = kCountsPerRevGearbox / kWheelCirc; // Meters per Revolution

		public static final double ksVolts = 0.22;
		public static final double kvVoltSecondsPerMeter = 0.3;
		public static final double kaVoltSecondsSquaredPerMeter = 0.01;

		public static final double kMaxSpeedMetersPerSecond = 1;
		public static final double kMaxAccelerationMetersPerSecondSquared = 0.7;

		public static final double kDistP = 0.15;
		public static final double kDistI = 0.0;
		public static final double kDistD = 0.0;

		public static final double kDistanceTolerance = 0.5;// inches

		public static final double kAngleRungAttached = 15;// degrees
	}

	public static final class ClimberConstants {
		public static final double kP = 0.2;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = -1.0;
		public static final double kMaxOutput = 1.0;

		public static final double kPulleyCirc = Math.PI * (20 / 25.4); // meters
		public static final int kEncoderResolution = 42; // not used, NEO's native units are rotations
		public static final double kGearBoxRatio = 12;
		public static final double kCountsPerRevGearbox = kEncoderResolution * kGearBoxRatio;
		public static final double kPosFactorIPC = kPulleyCirc / kCountsPerRevGearbox; // Meters per Revolution
		public static final double kPosFactorCPI = kCountsPerRevGearbox / kPulleyCirc; // Meters per Revolution

		public static final double kLowRung = 47.0;// inches
		public static final double kClearLowRung = 53.75;// inches
		public static final double kFullExtendPerpendicular = 66.0;// inches
		public static final double kFullExtendSwivel = 72.92;// inches
		public static final double kStow = 0.0;

		public static final double kDistanceTolerance = 0.5;// inches

		public static final double kPullUp = 0;// inches
	}

	public static final class ShooterConstants {
		public static final double kP = 0.00008;
		public static final double kI = 0.0000004;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = -.5;
		public static final double kMaxOutput = .5;

		public static final double kStopRPMs = 0.0;
		public static final double kMinRPM = -4540.0;
		public static final double kMaxRPM = 4540.8; // 2800 rpm when prototype tested 1-18-22

		public static final double kShooterShootRPMs = kMaxRPM * 0.45;

		public static final double kVelocityTolerance = 100.0; // rpms
	}

	public static final class IntakeConstants {
		public static final double kP = 0.00008;
		public static final double kI = 0.0000004;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = -.5;
		public static final double kMaxOutput = .5;

		public static final double kStopRPMs = 0.0;
		public static final double kMinRPM = -4540.0;
		public static final double kMaxRPM = 4540.8; // 2800 rpm when prototype tested 1-18-22

		public static final double kIntakeRPMs = kMaxRPM * 0.45;

		public static final double kVelocityTolerance = 100.0; // rpms
	}

	public static final class HopperConstants {
		public static final double kP = 0.00008;
		public static final double kI = 0.0000004;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = -.5;
		public static final double kMaxOutput = .5;

		public static final double kStopRPMs = 0.0;
		public static final double kMinRPM = -4540.0;
		public static final double kMaxRPM = 4540.8; // 2800 rpm when prototype tested 1-18-22

		public static final double kHopperRPMs = kMaxRPM * 0.45;

		public static final double kVelocityTolerance = 100.0; // rpms
	}
}
