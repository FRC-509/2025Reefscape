// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

		public static class MK4I { // MK4i level 2s
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

		public static final DCMotor kKrakenDcMotorProfile = new DCMotor(
			13.12, // TODO: Populate with MORE real data
			7.09, 
			366, 
			2, 
			kKrakenFreeSpeedRPM * 2 * Math.PI, 
			1);

		public static final ModuleConfig kModuleConfig = new ModuleConfig(
			MK4I.kWheelRadius,
			kMaxSpeed, 
			MK4I.wheelCOF,
			kKrakenDcMotorProfile,
			Constants.CurrentLimits.kSwerveModuleSupply, 
			4);

		public static record SwerveModuleConfiguration(
			int moduleNumber,
			int steerEncoderId,
			int steerMotorId,
			int driveMotorId,
			double steerEncoderOffset) {}

		// TODO: Immediately after reciving drivetrain, set these
		public static final SwerveModuleConfiguration kFrontRight = new SwerveModuleConfiguration(
			0,
			IDs.kFrontRightEncoder,
			IDs.kFrontRightSteer,
			IDs.kFrontRightDrive,
			154.423828);

		public static final SwerveModuleConfiguration kFrontLeft = new SwerveModuleConfiguration(
			1,
			IDs.kFrontLeftEncoder,
			IDs.kFrontLeftSteer,
			IDs.kFrontLeftDrive,
			101.337891);

		public static final SwerveModuleConfiguration kBackLeft = new SwerveModuleConfiguration(
			2,
			IDs.kBackLeftEncoder,
			IDs.kBackLeftSteer,
			IDs.kBackLeftDrive,
			178.066406);

		public static final SwerveModuleConfiguration kBackRight = new SwerveModuleConfiguration(
			3,
			IDs.kBackRightEncoder,
			IDs.kBackRightSteer,
			IDs.kBackRightDrive,
			258.925782);
	}

	public static class Elevator {
		// Gear ratio between the rotation of the motor and the rotation extending the elevator 
		public static final double kExtensionGearRatio = 0.0; // steer gear ratio
		public static final double kExtensionMagnetOffset = 0.0;
        public static final double kValidStateTolerance = 0;
		public static final double kMaxVelocity = 0; // Find desired units of setVelocity
        public static final double kMaxAcceleration = 0;
		public static final  Constraints kMotionProfileConstraints = new Constraints(kMaxVelocity, kMaxAcceleration);
	}

	public static class Arm {
		public static final double kRotationGearRatio = 0.0;
		public static final double kPivotMagnetOffset = 0.0;
		
		public static final double kIntakeSpeed = 0.0; // 0-1
		public static final double kValidRotationTolerance = 0;
	}

	public static class IDs {
		// Swerve Drive
		public static final int kFrontRightDrive = 0;
		public static final int kFrontRightSteer = 0;
		public static final int kFrontRightEncoder = 0;

		public static final int kFrontLeftDrive = 0;
		public static final int kFrontLeftSteer = 0;
		public static final int kFrontLeftEncoder = 0;
		
		public static final int kBackRightDrive = 0;
		public static final int kBackRightSteer = 0;
		public static final int kBackRightEncoder = 0;
		
		public static final int kBackLeftDrive = 0;
		public static final int kBackLeftSteer = 0;
		public static final int kBackLeftEncoder = 0;

		// Elevator
		public static final int kExtensionLeader = 0;
        public static final int kExtensionFollower = 0;

		// Arm
        public static final int kPivotMotor = 0;
		public static final int kPivotEncoder = 0;
		public static final int kIntakeMotor = 0;

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

		public static class Elevator {
			public static final double kExtensionP = 0;
			public static final double kExtensionI = 0;
			public static final double kExtensionD = 0;
		}

		public static class Arm {
			public static final double kRotationP = 0.0;
			public static final double kRotationI = 0.0;
			public static final double kRotationD = 0.0;

			public static final double kIntakeP = 0.0;
			public static final double kIntakeI = 0.0;
			public static final double kIntakeD = 0.0;
		}
	}

	public static class CurrentLimits {
		public static final double kSwerveModuleSupply = 35.0d;
		// SwerveStator ?

		public static final double kElevatorSupply = 0.0;
		public static final double kElevatorStator = 0.0;
		
		public static final double kArmSupply = 0.0;
		public static final double kArmStator = 0.0;

		public static final double kIntakeSupply = 0.0;
		public static final double kIntakeStator = 0.0;
	}

	public static class Field {
		public static final double kFieldLength = 16.54d; // Double check
	}
}
