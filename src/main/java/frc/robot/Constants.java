// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BezierPathGeneration.Obstacle;
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
	public static final String kCANIvore = "509CANIvore";

	public static class Operator {
		public static final double kStickDeadband = 0.1;
        public static final double kPrecisionMovementMultiplier = 0.3;
		public static final double kPrecisionRotationMultiplier = 0.2;
		public static final double kTriggerDeadband = 0.15;
	}

	public static class Chassis {
		public static final double kRobotWeight = 61.7d;
		public static final double kMOI = 5.2013;
		public static final double kOffsetToSwerveModule = 0.395;
		public static final double kKrakenFreeSpeedRPM = 6000.0d;
		public static final double kKrakenFreeSpeedRPS = kKrakenFreeSpeedRPM / 60.0d;
		public static final double kMaxSpeed = Conversions.falconToMPS(kKrakenFreeSpeedRPS, MK4I.kWheelCircumference,
			MK4I.kDriveGearRatio); // allegedly 4.7244
		
		public static class MK4I { // MK4i L2
			public static final double kWheelRadius = Units.inchesToMeters(2.0);
			public static final double kWheelCircumference = 2 * kWheelRadius * Math.PI; // 0.3192 meters
			public static final double kDriveGearRatio = 6.75 / 1;
			public static final double kAngleGearRatio = 150.0d / 7.0d;
			public static final double kCouplingRatio = 25.0d / 7.0d;
			public static final double wheelCOF = 1.0; // default placeholder value
		}

		public static final double kMaxAngularVelocity = kMaxSpeed
			/ (Math.hypot(Chassis.kOffsetToSwerveModule, Chassis.kOffsetToSwerveModule));
		// public static final double kMaxAngularAcceleration = 0.0;

		public static final DCMotor kKrakenDcMotorProfile = new DCMotor(
			12,
			7.09, 
			366, 
			2,
			kKrakenFreeSpeedRPS * 2 * Math.PI,
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

		public static final SwerveModuleConfiguration kFrontRight = new SwerveModuleConfiguration(
			0,
			IDs.kFrontRightEncoder,
			IDs.kFrontRightSteer,
			IDs.kFrontRightDrive,
			118.587891);

		public static final SwerveModuleConfiguration kFrontLeft = new SwerveModuleConfiguration(
			1,
			IDs.kFrontLeftEncoder,
			IDs.kFrontLeftSteer,
			IDs.kFrontLeftDrive,
			153.337891);

		public static final SwerveModuleConfiguration kBackLeft = new SwerveModuleConfiguration(
			2,
			IDs.kBackLeftEncoder,
			IDs.kBackLeftSteer,
			IDs.kBackLeftDrive,
			104.066406+1);

		public static final SwerveModuleConfiguration kBackRight = new SwerveModuleConfiguration(
			3,
			IDs.kBackRightEncoder,
			IDs.kBackRightSteer,
			IDs.kBackRightDrive,
			140.425782);
		public static final double kRobotWidth = 0;
        public static final double kSafePathingTolerance = 0;
        public static final double kValidPositionTolerance = 0;
        public static final double kValidHeadingTolerance = 0;
	}

	public static class Elevator {
		// Gear ratio between the rotation of the motor and the rotation extending the elevator
		public static final double kRotorToSensorRatio = 1.0 / 12.0;
        public static final double kSensorToMechanismRatio = 2 * Math.PI * Units.inchesToMeters(0.8);
		public static final double kExtensionMagnetOffset = 0.0;
		
        public static final double kValidExtensionTolerance = 0.05;
	}

	public static class Arm {
		public static final double kValidRotationTolerance = 0.04;

		public static final double kRotationGearRatio = 4.0 / 3.0;
        public static final double kSensorToMechanismRatio = 4 / 3;
        public static final double kRotorToSensorRatio = 12 * 84 / 24;
	}

	public static class Intake {
        public static final double kAlgaeIntakeVoltage = -5.5;
		public static final double kCoralIntakeVoltage = 6;

        public static final double kAlgaeTorqueCurrent = 20;
		public static final double kCoralTorqueCurrent = 18;

        public static final double kAlgaePassiveVoltage = -0.75;
        public static final double kCoralPassiveVoltage = 0.75;

        public static final double kAlgaeOutakeVoltage = 15.9;
        public static final double kCoralOutakeVoltage = -8.5;

		public static final double kCoralOutakeDelay = 0.75; // seconds
        public static final double kAlgaeOutakeDelay = 0.5;
		
        public static final double kIntakeClockPeriod = 0.2; // seconds
	}

	public static class Climber {
		public static final double kRotationGearRatio = 4.0 / 3.0;		
		public static final double kValidRotationTolerance = 0.005;
		public static final double kMaxRotationalSpeed = 0;
		public static final double kMinimumPositionDegrees = 0;
        public static final double kSensorToMechanismRatio = 0;

		public static final double kClimberAbsoluteMax = 142.0;
		public static final double kClimbReadyPosition = 136.0;
		public static final double kClimbFinalPosition = 43.5; //14
	}

	public static class IDs {
		// Swerve Drive
		public static final int kFrontRightDrive = 1;
		public static final int kFrontRightSteer = 2;
		public static final int kFrontRightEncoder = 9;

		public static final int kFrontLeftDrive = 3;
		public static final int kFrontLeftSteer = 4;
		public static final int kFrontLeftEncoder = 10;
		
		public static final int kBackRightDrive = 5;
		public static final int kBackRightSteer = 6;
		public static final int kBackRightEncoder = 11;
		
		public static final int kBackLeftDrive = 7;
		public static final int kBackLeftSteer = 8;
		public static final int kBackLeftEncoder = 12;

		// Elevator
		public static final int kExtensionLeader = 14; //Elevator left
        public static final int kExtensionFollower = 13;
        public static final int kExtensionEncoder = 18;
        public static final int kRangeSensor = 20;

		// Arm
        public static final int kPivotMotor = 15;
		public static final int kPivotEncoder = 19;
		public static final int kIntakeMotor = 16;

		public static final int kClimbMotor = 17;
		public static final int kClimbSolenoid = 8;
	}

	public static class Vision {
		public static final String leftLimelight = "limelight-left";
		public static final String rightLimelight = "limelight-right";

		public static final double kRotationAlignmentSpeedScalar = 5.45;
		public static final double kAlignmentRotationTolerance = 0;
        public static final double kAlignmentTranslationTolerance = 0.1;
        public static final double kCameraHeightFromGround = Units.inchesToMeters(39);
        public static final double kIntakeCameraAngleOffset = 0;
        public static final double kMaxTargetDistanceVariation = 10;
        public static final double kxFOV = 84;
        public static final double kyFOV = 56.4;

        public static class Pipeline {
            public static int AprilTags = 0;
            public static int NeuralNetwork = 1;
		}
	}

	public static class PathGeneration {
		public static final double kSafePathTolerance = 0.2; // m
	}

	public static class PIDConstants {
		public static class Drive {
			// TODO: Tune Me
			public static final double kDriveVelocityS = 0.124;
			public static final double kDriveVelocityV = 0.109;
			public static final double kDriveVelocityA = 0.0;

			public static final double kDriveVelocityP = 0.2;
			public static final double kDriveVelocityI = 3.0;
			public static final double kDriveVelocityD = 0.0;

			public static final double kSteerAngleP = 100.0;
			public static final double kSteerAngleI = 0.0;
			public static final double kSteerAngleD = 0.0;

			public static final double kHeadingPassiveP = 1.0;
			public static final double kHeadingPassiveI = 0.15;
			public static final double kHeadingPassiveD = 0.0;
			public static final double kHeadingAggressiveP = 4.5;
			public static final double kHeadingAggressiveI = 0.25;
			public static final double kHeadingAggressiveD = 0.0;
			public static final double kHeadingTimeout = 0.25;
			public static final double kMinHeadingCorrectionSpeed = 0.1;
		}

		public static class Elevator {
			public static final double kExtensionP = 1.9;
			public static final double kExtensionI = 1.0;
			public static final double kExtensionD = 0;
		}

		public static class Arm {
			public static final double kRotationP = 2.8;
			public static final double kRotationI = 0.32;
			public static final double kRotationD = 0.0;

			public static final double kIntakeP = 0.03;
			public static final double kIntakeI = 0.01;
			public static final double kIntakeD = 0.0;
		}

        public static class Climber {
			public static final double kRotateP = 1.0;
			public static final double kRotateI = 0.0;
			public static final double kRotateD = 0.0;
		}
	}

	public static class CurrentLimits {
		public static final double kSwerveModuleSupply = 35.0d;
		// SwerveStator ?

		public static final double kElevatorSupply = 35.0;
		public static final double kElevatorStator = 120.0;
		
		public static final double kArmSupply = 35.0;
		public static final double kArmStator = 120.0;

		public static final double kIntakeSupply = 35.0;
		public static final double kIntakeStator = 120.0;

        public static final double kClimbSupply = 40;
        public static final double kClimbStator = 120;
	}

	public static class Field {
		public static final double kFullFieldLength = 16.54d; // Double check
		
		public static final double kReefRadius = Units.inchesToMeters(77.5d)/2;
		public static final double kReefOffsetX = Units.inchesToMeters(0.0); // TODO: Find
		public static final double kReefOffsety = Units.inchesToMeters(144.0d) + kReefRadius;

		public static final double kBargeLength = Units.inchesToMeters(46.0d);
		public static final double kBargeWidth = Units.inchesToMeters(146.5d);
		public static final double kBargeDistance = kReefOffsety + kReefRadius + Units.inchesToMeters(88);

		public static final Obstacle reef_BLUE_ALLIANCE = new Obstacle(kReefOffsetX, kReefOffsety, kReefRadius);
		public static final Obstacle reef_RED_ALLIANCE = new Obstacle(kReefOffsetX, kReefOffsety, kReefRadius);

        public static final double kFieldWidth = 8; // TODO: find a real number
	}

	public static double tunableNumber(String name, double defaultValue){
		return SmartDashboard.getNumber(name, defaultValue);
	}
}
