// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.util.math.Conversions;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final String kRio = "rio";
	public static final String kCANIvore = "509CANIvore"; //TODO: Make sure to flash 

	public static class Operator {
		public static final double kStickDeadband = 0.1;
        public static final double kPrecisionMovementMultiplier = 0.3; // TODO: Tune to what feels good
		public static final double kPrecisionRotationMultiplier = 0.3;
	}

	public static class Chassis {
		public static final double kRobotWeight = 0.0; // kg, incl bumpers and battery
		public static final double kOffsetToSwerveModule = Units.inchesToMeters(10.375);
		public static final double kFalconFreeSpeedRPS = 6380.0d / 60.0d; // div 60
		public static final double kKrakenFreeSpeedRPM = 6000;
		public static final double kMaxSpeed = Conversions.falconToMPS(kFalconFreeSpeedRPS, MK4I.kWheelCircumference,
			MK4I.kDriveGearRatio);

		public static class MK4I { // MK4i level 2s, should be same
			public static final double kWheelRadius = Units.inchesToMeters(2.0);
			public static final double kWheelCircumference = 2 * kWheelRadius * Math.PI; // 0.3192 meters
			public static final double kDriveGearRatio = 425.0d / 63.0d;
			public static final double kAngleGearRatio = 150.0d / 7.0d;
			public static final double kCouplingRatio = 25.0d / 7.0d;
			public static final double wheelCOF = 0.0; // TODO: find later
		}

		public static final double kMaxAngularVelocity = kMaxSpeed
			/ (Math.hypot(Chassis.kOffsetToSwerveModule, Chassis.kOffsetToSwerveModule));
		// public static final double kMaxAngularAcceleration = 0.0;

		public static record SwerveModuleConfiguration(
			int moduleNumber,
			int steerEncoderId,
			int steerMotorId,
			int driveMotorId,
			double steerEncoderOffset) {}

		// TODO: Immediately after reciving drivetrain, set these
		public static final SwerveModuleConfiguration kFrontRight = new SwerveModuleConfiguration(
			0,
			0,
			0,
			0,
			154.423828);

		public static final SwerveModuleConfiguration kFrontLeft = new SwerveModuleConfiguration(
			1,
			0,
			0,
			0,
			101.337891);

		public static final SwerveModuleConfiguration kBackLeft = new SwerveModuleConfiguration(
			2,
			0,
			0,
			0,
			178.066406);

		public static final SwerveModuleConfiguration kBackRight = new SwerveModuleConfiguration(
			3,
			0,
			0,
			0,
			258.925782);
	}

	public static class PIDConstants {
		public static class Drive {
			// TODO: Change all values
			public static final double kDriveVelocityS = 0.124;
			public static final double kDriveVelocityV = 0.109;
			public static final double kDriveVelocityA = 0.0;

			public static final double kDriveVelocityP = 0.2;
			public static final double kDriveVelocityI = 3.0;
			public static final double kDriveVelocityD = 0.0;

			public static final double kSteerAngleP = 100.0;
			public static final double kSteerAngleI = 0.0;
			public static final double kSteerAngleD = 0.0;

			// TODO: Tune Me!
			public static final double kHeadingPassiveP = 8.0;
			public static final double kHeadingPassiveI = 0.15;
			public static final double kHeadingPassiveD = 0.4;
			public static final double kHeadingAggressiveP = 11.5;
			public static final double kHeadingAggressiveI = 0.25;
			public static final double kHeadingAggressiveD = 0.8;
			public static final double kHeadingTimeout = 0.25;
			public static final double kMinHeadingCorrectionSpeed = 0.05;
		}
	}

	public static class CurrentLimits {
		public static final double kSwerveModule = 35.0d;
	}

	public static class Field {
		public static final double kFieldLength = 16.54d; // Double check
	}
}
