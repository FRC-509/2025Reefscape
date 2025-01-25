package frc.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import frc.robot.Constants;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.util.LoggablePID;
import frc.robot.util.PigeonWrapper;
import frc.robot.util.ThinNT;
import frc.robot.util.math.GeometryUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.redstorm509.stormkit.math.Interpolator;

public class SwerveDrive extends SubsystemBase {

	/*
	 * The order of each vector corresponds to the index of the swerve module inside
	 * the swerveModules array. Everything looks weird because WPILib does
	 * everything like this:
	 * ^
	 * | X+
	 * ----> Y-
	 * module 1 (+, +) |--f--| module 0 (+, -)
	 * module 2 (-, +) |--b--| module 3 (-, -)
	 */
	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
				new Translation2d(+Constants.Chassis.kOffsetToSwerveModule, -Constants.Chassis.kOffsetToSwerveModule),
				new Translation2d(+Constants.Chassis.kOffsetToSwerveModule, +Constants.Chassis.kOffsetToSwerveModule),
				new Translation2d(-Constants.Chassis.kOffsetToSwerveModule, +Constants.Chassis.kOffsetToSwerveModule),
				new Translation2d(-Constants.Chassis.kOffsetToSwerveModule, -Constants.Chassis.kOffsetToSwerveModule));

	public SwerveModule[] swerveModules;
	public SwerveDriveOdometry odometry;
	public SwerveDrivePoseEstimator poseEstimator;
	public Field2d field2d;
	private PigeonWrapper pigeon;

	private Interpolator headingInterplator;

	private Timer manualRotationTimer;

	private double prevTime = -1.0d;
	private double targetHeading;
	private LoggablePID headingPassive = new LoggablePID(Constants.PIDConstants.Drive.kHeadingPassiveP, Constants.PIDConstants.Drive.kHeadingPassiveI,
			Constants.PIDConstants.Drive.kHeadingPassiveD);
	private LoggablePID headingAggressive = new LoggablePID(Constants.PIDConstants.Drive.kHeadingAggressiveP,
			Constants.PIDConstants.Drive.kHeadingAggressiveI, Constants.PIDConstants.Drive.kHeadingAggressiveD);

	private boolean alwaysOmitRotationalCorrection = false;
	private double yawFromWhenWeLastSetTargetHeading = 0.0d;

	private double simHeading = 0.0d;
	private double prevRotOutput = 0.0d;
	private Limelight shooterCamera;
	private StructArrayPublisher<SwerveModuleState> moduleStatePublisher;
	private StructPublisher<Pose2d> odometryPublisher;
	private StructPublisher<Pose2d> poseEstimatePublisher;
	
	/*
	private HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
			new PIDConstants(5.0, 0, 0), // Translation constants
			new PIDConstants(5.0, 0, 0), // Rotation constants
			Constants.Chassis.kMaxSpeed,
			Constants.Chassis.kOffsetToSwerveModule * Math.sqrt(2),
			new ReplanningConfig(true, true));
	private PathFollowingController p = new PathFollowingController(
		new PIDConstants(5.0, 0, 0), // Translation constants
		new PIDConstants(5.0, 0, 0), // Rotation constants
		Constants.Chassis.kMaxSpeed,
		Constants.Chassis.kOffsetToSwerveModule * Math.sqrt(2),
		new ReplanningConfig(true, true),
	);
	*/

	public SwerveDrive(PigeonWrapper pigeon, Limelight shooterCamera) {
		this.manualRotationTimer = new Timer();
		manualRotationTimer.start();

		this.pigeon = pigeon;
		this.shooterCamera = shooterCamera;
		this.headingInterplator = new Interpolator(Constants.Chassis.kMaxAngularVelocity);
		this.targetHeading = 0.0;

		pigeon.setYaw(0.0d);

		swerveModules = new SwerveModule[] {
				new SwerveModule(Constants.Chassis.kFrontRight),
				new SwerveModule(Constants.Chassis.kFrontLeft),
				new SwerveModule(Constants.Chassis.kBackLeft),
				new SwerveModule(Constants.Chassis.kBackRight),
		};
		
		field2d = new Field2d();
		/* possible TODO: add autonomous gamestate
		field2d.getObject("Coral").setPoses(
				new Pose2d(),
				new Pose2d(),
				new Pose2d(),
				new Pose2d(),
				new Pose2d(),
				new Pose2d(),
				new Pose2d(),
				new Pose2d());
		*/


		// end

		odometry = new SwerveDriveOdometry(kinematics, getYaw(), getModulePositions());
		poseEstimator = new SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), new Pose2d());
		poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, Math.toRadians(7)));
		AutoBuilder.configure(
				this::getEstimatedPose, //
				this::resetOdometry, //
				this::getChassisSpeeds, //
				this::setChassisSpeeds, //
				new PathFollowingController(){
					public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
						return getChassisSpeeds(); // TODO: assert this works?
					}
					public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) { resetOdometry(currentPose); }
					public boolean isHolonomic() { return true; }
				},
				new RobotConfig(
					Constants.Chassis.kRobotWeight,
					0.0, // TODO: get later
					Constants.Chassis.kModuleConfig,
					// TODO: which way is forward, which way does it think is forwards
					// Front Left, Front Right, Back Left, Back Right 
					new Translation2d(+Constants.Chassis.kOffsetToSwerveModule, -Constants.Chassis.kOffsetToSwerveModule),
					new Translation2d(+Constants.Chassis.kOffsetToSwerveModule, +Constants.Chassis.kOffsetToSwerveModule),
					new Translation2d(-Constants.Chassis.kOffsetToSwerveModule, +Constants.Chassis.kOffsetToSwerveModule),
					new Translation2d(-Constants.Chassis.kOffsetToSwerveModule, -Constants.Chassis.kOffsetToSwerveModule)),
				() -> { //
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					Alliance alliance = getAlliance();
					return alliance == DriverStation.Alliance.Red;
				},
				this); //
		// PathPlannerLogging.setLogActivePathCallback((poses) ->
		// field2d.getObject("path").setPoses(poses));
		Shuffleboard.getTab("Robot Field Position").add(field2d);

		headingPassive.setIntegratorRange(-180, 180);
		headingAggressive.setIntegratorRange(-180, 180);
		headingPassive.setTolerance(0.25);
		headingAggressive.setTolerance(0.25);

		moduleStatePublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("module-states", SwerveModuleState.struct).publish();
		odometryPublisher = NetworkTableInstance.getDefault()
				.getStructTopic("odometry-pose", Pose2d.struct).publish();
		poseEstimatePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("estimated-pose", Pose2d.struct).publish();
		/*-
		SmartDashboard.putNumber("HeadingPassiveP", Constants.kHeadingPassiveP);
		SmartDashboard.putNumber("HeadingPassiveI", Constants.kHeadingPassiveI);
		SmartDashboard.putNumber("HeadingPassiveD", Constants.kHeadingPassiveD);
		
		SmartDashboard.putNumber("HeadingAggressiveP", Constants.kHeadingAggressiveP);
		SmartDashboard.putNumber("HeadingAggressiveI", Constants.kHeadingAggressiveI);
		SmartDashboard.putNumber("HeadingAggressiveD", Constants.kHeadingAggressiveD);
		
		SmartDashboard.putData("Set Heading to 0", new InstantCommand(() -> this.setTargetHeading(0), this));
		SmartDashboard.putData("Set Heading to 180", new InstantCommand(() -> this.setTargetHeading(180), this));
		*/
	}

	public Alliance getAlliance() {
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			return alliance.get();
		}
		if (DriverStation.isFMSAttached()) {
			DriverStation.reportError("Failed to get alliance color from FMS!", false);
		} else {
			DriverStation.reportError("Failed to get alliance color from Driver Station!", false);
		}
		return Alliance.Blue;
	}

	public void drive(Translation2d translationMetersPerSecond, double rotationRadiansPerSecond, boolean fieldRelative,
			boolean omitRotationCorrection) {
		if (alwaysOmitRotationalCorrection) {
			omitRotationCorrection = true;
		}
		double dt = Timer.getFPGATimestamp() - prevTime;
		if (prevTime < 0) {
			dt = 0.02;
		}

		headingInterplator.setPoint(rotationRadiansPerSecond);

		double rotationOutput;

		double interpolatedRotation = headingInterplator.update(dt);

		boolean hasRotationInput = Math.abs(rotationRadiansPerSecond) > 0.01;

		if (hasRotationInput) {
			manualRotationTimer.reset();
		}

		double speed = Math.hypot(translationMetersPerSecond.getX(),
				translationMetersPerSecond.getY());
		
		if ((speed != 0 && speed < Constants.PIDConstants.Drive.kMinHeadingCorrectionSpeed) || omitRotationCorrection || hasRotationInput
				|| manualRotationTimer.get() < Constants.PIDConstants.Drive.kHeadingTimeout) {
			// if (hasRotationInput || timer.get() < Constants.kHeadingTimeout) {
			setTargetHeading(getYaw().getDegrees());
			headingPassive.reset();
			headingAggressive.reset();
			// }
			rotationOutput = interpolatedRotation;
		} else {
			double delta = getYaw().getDegrees() - targetHeading;
			if (delta > 180.0d) {
				delta -= 360.0d;
			}
			if (delta < -180.0d) {
				delta += 360.0d;
			}

			double overallDrift = yawFromWhenWeLastSetTargetHeading - targetHeading;
			if (overallDrift > 180.0d) {
				overallDrift -= 360.0d;
			}
			if (overallDrift < -180.0d) {
				overallDrift += 360.0d;
			}

			double aggressiveOutput = headingAggressive.calculate(delta);
			double passiveOutput = headingPassive.calculate(delta);

			// Remove the D term from passive if its output is less than 1
			if (Math.abs(headingPassive.getLastDOutput()) < 1.0) {
				passiveOutput = headingPassive.getLastPOutput() + headingPassive.getLastIOutput();
			}

			double outputDegrees = Math.abs(overallDrift) > 10.0d ? aggressiveOutput : passiveOutput;
			// double outputDegrees = passiveOutput;

			/*-
			SmartDashboard.putNumber("HeadingPassivePOutput", headingPassive.getLastPOutput());
			SmartDashboard.putNumber("HeadingPassiveIOutput", headingPassive.getLastIOutput());
			SmartDashboard.putNumber("HeadingPassiveDOutput", headingPassive.getLastDOutput());
			SmartDashboard.putNumber("HeadingAggressivePOutput", headingAggressive.getLastPOutput());
			SmartDashboard.putNumber("HeadingAggressiveIOutput", headingAggressive.getLastIOutput());
			SmartDashboard.putNumber("HeadingAggressiveDOutput", headingAggressive.getLastDOutput());
			*/

			rotationOutput = Math.toRadians(outputDegrees);
		}

		prevRotOutput = rotationOutput;
		SwerveModuleState[] moduleStates;

		if (fieldRelative) {
			moduleStates = kinematics
					.toSwerveModuleStates(ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
							translationMetersPerSecond.getX(),
							translationMetersPerSecond.getY(),
							rotationOutput,
							getYaw()), dt));
		} else {
			moduleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(
					translationMetersPerSecond.getX(),
					translationMetersPerSecond.getY(),
					rotationOutput));
		}

		SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
				Constants.Chassis.kMaxSpeed);

		for (SwerveModule mod : swerveModules) {
			mod.setDesiredState(moduleStates[mod.moduleNumber], true);
		}

		prevTime = Timer.getFPGATimestamp();
	}

	public void enterXStance() {
		swerveModules[0].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(45.0d)), false);
		swerveModules[1].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(135.0d)), false);
		swerveModules[2].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(45.0d)), false);
		swerveModules[3].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(135.0d)), false);
	}

	public void stopModules() {
		for (SwerveModule module : swerveModules) {
			module.setDesiredState(new SwerveModuleState(0, module.getAngle()), false);
		}
	}

	public void setTargetHeading(double heading) {
		yawFromWhenWeLastSetTargetHeading = getYaw().getDegrees() % 360.0;
		targetHeading = heading % 360.0d;
	}

	public void toggleHeadingCorrection() {
		alwaysOmitRotationalCorrection = !alwaysOmitRotationalCorrection;
		headingAggressive.reset();
		headingPassive.reset();
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	// Used strictly for PathPlanner in autonomous
	public void setChassisSpeeds(ChassisSpeeds robotRelativeSpeeds) {
		ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
		SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Chassis.kMaxSpeed);
		prevRotOutput = targetSpeeds.omegaRadiansPerSecond;

		for (SwerveModule mod : swerveModules) {
			mod.setDesiredState(targetStates[mod.moduleNumber], true);
		}
	}

	/**
	 * Generates a command to reset the odometer to the given pose. Must be relative
	 * to the BLUE ALLIANCE origin!
	 */
	public Command resetOdometryCmd(Pose2d pose) {
		return Commands.runOnce(
				() -> {

					boolean flip = false;
					Alliance alliance = getAlliance();
					flip = alliance == DriverStation.Alliance.Red;
					if (flip) {
						Pose2d flipped = GeometryUtils.flipFieldPose(pose);
						pigeon.setYaw(flipped.getRotation().getDegrees());
						resetOdometry(new Pose2d(flipped.getTranslation(), getYaw()));
					} else {
						pigeon.setYaw(pose.getRotation().getDegrees());
						resetOdometry(new Pose2d(pose.getTranslation(), getYaw()));
					}
				}, this);
	}

	public double jankFlipHeading(double heading) {
		boolean flip = false;
		Alliance alliance = getAlliance();
		flip = alliance == DriverStation.Alliance.Red;
		if (flip) {
			return 180.0 - heading;
		} else {
			return heading;
		}
	}

	public Pose2d getRawOdometeryPose() {
		return odometry.getPoseMeters();
	}

	public Pose2d getEstimatedPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(getYaw(), getModulePositions(), pose);
		poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
	}

	public SwerveModule[] getModules() {
		return swerveModules;
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : swerveModules) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : swerveModules) {
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public Rotation2d getYaw() {
		if (RobotBase.isSimulation()) {
			return Rotation2d.fromRadians(simHeading);
		}
		return pigeon.getRotation2d();
	}

	public void resetSimState() {
		simHeading = 0.0d;
		targetHeading = 0.0d;
		resetOdometry(new Pose2d());
		for (SwerveModule mod : swerveModules) {
			mod.resetSimState();
		}
	}

	@Override
	public void simulationPeriodic() {
		simHeading += prevRotOutput * 0.02;
		for (SwerveModule mod : swerveModules) {
			mod.simPeriodic();
		}
	}

	@Override
	public void periodic() {
		odometry.update(getYaw(), getModulePositions());
		poseEstimator.update(getYaw(), getModulePositions());

		if (shooterCamera.getTV()) {
			double[] botPoseData = shooterCamera.getBotPoseBlue();
			int numberOfTags = (int) botPoseData[7];
			if (numberOfTags >= 2) {
				Pose2d visionPose = Limelight.toPose2D(botPoseData);
				double timestamp = Timer.getFPGATimestamp() - (shooterCamera.getLatency_Pipeline() / 1000.0)
						- (shooterCamera.getLatency_Capture() / 1000.0);
				poseEstimator.addVisionMeasurement(visionPose, timestamp);
			}
		}
		field2d.setRobotPose(getEstimatedPose());

		SmartDashboard.putNumber("yaw", getYaw().getDegrees());
		SmartDashboard.putBoolean("Heading Correction Enabled?", !alwaysOmitRotationalCorrection);

		ThinNT.putNumber("x-velocity", getChassisSpeeds().vxMetersPerSecond);
		ThinNT.putNumber("y-velocity", getChassisSpeeds().vyMetersPerSecond);
		ThinNT.putNumber("yaw-velocity", pigeon.getAngularVelocityZWorld());
		ThinNT.putNumber("target-heading", targetHeading);

		moduleStatePublisher.set(getModuleStates());
		odometryPublisher.set(getRawOdometeryPose());
		poseEstimatePublisher.set(getEstimatedPose());
	}
}