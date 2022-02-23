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

		public static final int kShooterMotor = 30;
		public static final int kInjectorMotor = 31;

		public static final int kIntakeMotor = 40;

		public static final int kHopperMotor = 50;
	}

	public static final class PneumaticChannelConstants {
		public static final int kClimbLeftExtend = 0;
		public static final int kClimbLeftRetract = 1;
		public static final int kClimbRightExtend = 2;
		public static final int kClimbRightRetract = 3;

		public static final int kLatchOpen = 4;
		public static final int kLatchClose = 5;

		public static final int kIntakeExtend = 6;
		public static final int kIntakeRetract = 7;
	}

	public static final class DIOChannelConstants {
		public static final int kIntakeEntering = 0;
		public static final int kHopperEntering = 1;
		public static final int kHopperExiting = 2;
		public static final int kInjectorEntering = 3;
		public static final int kInjectorExiting = 4;
	}

	public static final class PWMChannelConstants {
		public static final int kShooterLeftServo = 0;
		public static final int kShooterRightServo = 1;
	}

	public static final class PDPChannelConstants {
		public static final int kClimberRight = 13;
		public static final int kClimberLeft = 14;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;

		public static final double kRumbleDelay = 1.0;
	}

	public static final class AnalogIOConstants {
		public static final int kHiPressureChannel = 0;
		public static final int kLoPressureChannel = 1;

		public static final double kInputVoltage = 5.0;
	}

	public static final class ChassisConstants {
		// Constants for Drive PIDs
		public static final double kP = 0.000000000005;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = -1.0;
		public static final double kMaxOutput = 1.0;

		public static final double kTrackWidth = Units.inchesToMeters(26.341); // meters
		public static final double kWheelCirc = Units.inchesToMeters(Math.PI * 8.0); // meters
		public static final int kEncoderResolution = 1; // not used, NEO's native units are rotations
		public static final double kGearBoxRatio = 10.71;
		public static final double kPosFactor = kWheelCirc / kGearBoxRatio; // Meters per Revolution
		public static final double kVelFactor = kWheelCirc / kGearBoxRatio / 60.0; // Meters per Second
		public static final double kCountsPerRevGearbox = kEncoderResolution * kGearBoxRatio;

		public static final double kPosFactorIPC = kWheelCirc / kCountsPerRevGearbox; // Meters per Revolution
		public static final double kPosFactorCPI = kCountsPerRevGearbox / kWheelCirc; // Meters per Revolution

		public static final double ksVolts = 0.22;
		public static final double kvVoltSecondsPerMeter = 0.3;
		public static final double kaVoltSecondsSquaredPerMeter = 0.01;

		// Example value only - as above, this must be tuned for your drive!
		public static final double kPDriveVel = 0.5;

		public static final double kMaxSpeedMetersPerSecond = 1.0;
		public static final double kMaxAccelerationMetersPerSecondSquared = 0.7;

		public static final double kDistP = 0.15;
		public static final double kDistI = 0.0;
		public static final double kDistD = 0.0;

		public static final double kDistanceTolerance = 0.5;// inches

		public static final double kAngleRungAttached = 15.0;// degrees

		// Reasonable baseline values for a RAMSETE follower in units of meters and
		// seconds
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;
	}

	public static final class ClimberConstants {
		public static final double kP = 0.25;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = -1.0;
		public static final double kMaxOutput = 1.0;

		public static final double kRopeDia = 0.125; // inches add to Circ calc
		public static final double kPulleyCirc = Math.PI * ((20.2 / 25.4) + kRopeDia); // inches
		public static final int kEncoderResolution = 1; // not used, NEO's native units are rotations
		public static final int kGearBoxRatio = 12;
		public static final double kCountsPerRevGearbox = kEncoderResolution * kGearBoxRatio;
		public static final double kPosFactorIPC = kPulleyCirc / kCountsPerRevGearbox; // inches per count
		public static final double kPosFactorCPI = kCountsPerRevGearbox / kPulleyCirc; // counts per inch

		// all climber measurements are from floor to underside of hook and floor to top
		// of rung
		public static final double kFloor2Hook = 10.4 + 34.03; // inches from floor to top of outer climber tube
		public static final double kClearUnder = 3.0; // inches below top of rung to clear
		public static final double kEngageOver = 3.0; // inches avobe top of rung to engage

		public static final double kLowRung = 48.75; // inches above floor per rules
		public static final double kClearLowRung = kLowRung - kFloor2Hook - kClearUnder; // inches
		public static final double kEngageLowRung = kLowRung - kFloor2Hook + kEngageOver; // inches
		public static final double kMidRung = 60.25; // inches aboave floor per rules
		public static final double kClearMidRung = kMidRung - kFloor2Hook - kClearUnder; // inches
		public static final double kEngageMidRung = kMidRung - kFloor2Hook + kEngageOver; // inches
		public static final double kEngageHighTrav = 66.0 - kFloor2Hook; // inches to position for engage
																			// high/traverse rungs
		public static final double kLatchHighTrav = 62.0 - kFloor2Hook; // inches to position for latch high/traverse
																		// rungs
		// public static final double kFullExtendPerpendicular = 66.0; // inches
		// public static final double kFullExtendSwivel = 72.92;// inches
		public static final double kPullUpLatch = 3.0; // inches to latch climber
		public static final double kPullUpClear = 6.0; // inches to unhook while latched
		public static final double kStow = 0.0;
		public static final double kOneRev = kPulleyCirc;

		public static final double kDistanceTolerance = 0.25; // inches

		public static final double kLatchDelay = 1.0; // seconds
		public static final double kSwivelDelay = 3.0; // seconds
		public static final double kInitDelay = 0.25; // seconds
		public static final double kInitSafety = 10.0; // seconds

		public static final double kMaxAmps = 10.0;
		public static final double kInitSpeed = 0.3;
	}

	public static final class ShooterConstants {
		public static final double kShootP = 0.00008;
		public static final double kShootI = 0.0000004;
		public static final double kShootD = 0.0;
		public static final double kShootIz = 0.0;
		public static final double kShootFF = 0.0;
		public static final double kShootMinOutput = -1.0;
		public static final double kShootMaxOutput = 1.0;

		public static final double kInjectP = 0.00008;
		public static final double kInjectI = 0.0000004;
		public static final double kInjectD = 0.0;
		public static final double kInjectIz = 0.0;
		public static final double kInjectFF = 0.0;
		public static final double kInjectMinOutput = -1.0;
		public static final double kInjectMaxOutput = 1.0;

		public static final double kStopRPMs = 0.0;
		public static final double kMinRPM = -4540.0;
		public static final double kMaxRPM = 4540.8; // 2800 rpm when prototype tested 1-18-22

		public static final double kShooterShootRPMs = kMaxRPM * 0.45;
		public static final double kInjectorerShootRPMs = kMaxRPM * 0.45;

		public static final double kShootVelocityTolerance = 100.0; // rpms
		public static final double kInjectVelocityTolerance = 100.0; // rpms
	}

	public static final class IntakeConstants {
		public static final double kP = 0.00008;
		public static final double kI = 0.0000004;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = -0.5;
		public static final double kMaxOutput = 0.5;

		public static final double kStopRPMs = 0.0;
		public static final double kMinRPM = -4540.0;
		public static final double kMaxRPM = 4540.8; // 2800 rpm when prototype tested 1-18-22

		public static final double kIntakeRPMs = kMaxRPM * 0.45;

		public static final double kVelocityTolerance = 100.0; // rpms
	}

	public static final class HopperConstants {
		public static final double kP = 0.00008;
		public static final double kI = 0.0000004;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = -0.5;
		public static final double kMaxOutput = 0.5;

		public static final double kStopRPMs = 0.0;
		public static final double kMinRPM = -4540.0;
		public static final double kMaxRPM = 4540.8; // 2800 rpm when prototype tested 1-18-22

		public static final double kHopperRPMs = kMaxRPM * 0.45;

		public static final double kVelocityTolerance = 100.0; // rpms
	}
}
