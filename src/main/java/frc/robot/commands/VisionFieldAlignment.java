package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Limelight;

import java.util.function.DoubleSupplier;


public class VisionFieldAlignment extends Command {

	private SwerveDrive swerve;
	private Limelight limelight;

	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;
	private DoubleSupplier rotationSupplier;

	private int targetTagID;
	private boolean isReefTag;

	private Translation3d RobotToTag;
	private Translation2d outputTranslation;


	// Meant to be an "isDownBind" command
	public VisionFieldAlignment(
			SwerveDrive swerve,
			Limelight limelight,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier rotationSupplier) {
		this.swerve = swerve;
		this.limelight = limelight;

		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.rotationSupplier = rotationSupplier;

		addRequirements(swerve);
	}

    /**
     * collects desired offset position
     * @param TagID
     * @return part of pose should be the desired translational offset
     */
	public Pose2d getAlignmentOffset(int TagID) {

		double desiredRotation;

		// TODO: find offsets
		switch (TagID) {

			// Processor
			case 3:
			case 16:
				// Rotate to be parallel with the 
				desiredRotation = Math.toRadians(-limelight.getTX()) * Constants.Vision.kRotationAlignmentSpeedScalar;			

				SmartDashboard.putNumber("desiredRotationAA",
						Math.toDegrees(MathUtil.clamp(Math.toRadians(desiredRotation), -Constants.Chassis.kMaxAngularVelocity,
								Constants.Chassis.kMaxAngularVelocity)));

				return new Pose2d(new Translation2d(0, 0), new Rotation2d(desiredRotation));

            // Coral Station
            case 1:
            case 2:
			case 12: 
			case 13: 
				return new Pose2d();

			// Reef
			case 6:
			case 7:
			case 8:
			case 9:
            case 10:
            case 11:
            case 17:
            case 18:
            case 19:
            case 20:
            case 21:
            case 22:
				return new Pose2d(new Translation2d(0, 0), new Rotation2d());

            // Friendly Alliance Barge
            case 5:
            case 14:
                return new Pose2d();

            // Opposing Alliance Barge
            case 4:
            case 15:
            return new Pose2d();

            // No Tag Found
			default:
				return new Pose2d(); // empty pose
		}
	}

	@Override
	public void execute() {
		// if there is no valid tag, then default driver behaivor is given
		if (!limelight.getTV()){
			SmartDashboard.putBoolean("Autonomous Lock On", false);
			// lights.setColor(ColorCode.AutoTargetLost);
			swerve.drive(
				new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).times(Constants.Chassis.kMaxSpeed),
				rotationSupplier.getAsDouble() * Constants.Chassis.kMaxAngularVelocity * 0.5, // Reduce rotation for precision
				true,
				false);
			return;
		}

		targetTagID = (int) limelight.getFiducialID();
		SmartDashboard.putBoolean("Autonomous Lock On", true);
		SmartDashboard.putNumber("Targeted April Tag", targetTagID);
		// lights.setColor(ColorCode.AutoTargetFound);

		// The desired translation
		Pose3d TagToRobotPose = Limelight.toPose3D(limelight.getBotPose_TargetSpace());
		RobotToTag = new Translation3d(-TagToRobotPose.getZ(), -TagToRobotPose.getX(), -TagToRobotPose.getY());

		Pose2d offsetPose = getAlignmentOffset(targetTagID);

		outputTranslation = RobotToTag.toTranslation2d().minus(offsetPose.getTranslation());

		if (offsetPose.equals(new Pose2d())) return; // checks if LL has valid tag
		if (!offsetPose.getTranslation().equals(new Translation2d())) {
			swerve.setTargetHeading(offsetPose.getRotation().getDegrees());

			// possible wait for target heading to be near reached if needed
			swerve.drive(
				new Translation2d(
					MathUtil.clamp(outputTranslation.getX(), -Constants.Chassis.kMaxSpeed, Constants.Chassis.kMaxSpeed),
					MathUtil.clamp(outputTranslation.getY(), -Constants.Chassis.kMaxSpeed, Constants.Chassis.kMaxSpeed)),
				0.0,
				false, // check for issues with rotation & field relative
				false);
		} else {
			swerve.drive(
				new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).times(Constants.Chassis.kMaxSpeed),
				MathUtil.clamp(offsetPose.getRotation().getRadians(), -Constants.Chassis.kMaxAngularVelocity,
					Constants.Chassis.kMaxAngularVelocity),
				true,
				true);
		}


		
		// if valid tag but no translation, sets desired rotation with operator
		// movement, otherwise full operator control
		swerve.drive(
				new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).times(Constants.Chassis.kMaxSpeed),
				rotationSupplier.getAsDouble() * Constants.Chassis.kMaxAngularVelocity * 0.5,
				true,
				false);
	}

	public void defaultOffsetBehaivior(){
		
	}

	public void reefOffset(){

	}

	@Override
	public boolean isFinished() {
		if (DriverStation.isAutonomous() && limelight.getTX() < 0.5d) {
			return true;
		}
		
		return MathUtil.isNear(0, outputTranslation.getX(), Constants.Vision.kAlignmentTranslationTolerance) &&
			MathUtil.isNear(0, outputTranslation.getY(), Constants.Vision.kAlignmentTranslationTolerance) &&
			MathUtil.isNear(getAlignmentOffset(targetTagID).getRotation().getDegrees(), swerve.getYaw().getDegrees(), Constants.Vision.kAlignmentRotationTolerance);
	}

	@Override
	public void end(boolean wasInterrupted) {
		//lights.setDefault();
		SmartDashboard.putBoolean("Autonomous Lock On", false);
		swerve.drive(new Translation2d(0, 0), 0, true, false);
		swerve.setTargetHeading(swerve.getYaw().getDegrees());
		if (DriverStation.isAutonomous()) {
			swerve.stopModules();
		}
	}
}
