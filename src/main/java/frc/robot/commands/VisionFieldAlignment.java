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
import frc.robot.subsystems.vision.LimelightHelpers;

import java.util.function.DoubleSupplier;


public class VisionFieldAlignment extends Command {

	private SwerveDrive swerve;
	private String highLimelight;
	private String lowLimelight;
	private String activeLimelight;

	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;
	private DoubleSupplier rotationSupplier;

	private int targetTagID;
	private boolean isReefTag;

	private Translation2d RobotToTag;
	private Translation2d outputTranslation;


	// Meant to be an "isDownBind" command
	public VisionFieldAlignment(
			SwerveDrive swerve,
			String highLimelight,
			String lowLimelight,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier rotationSupplier) {
		this.swerve = swerve;
		this.highLimelight = highLimelight;
		this.lowLimelight = lowLimelight;

		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.rotationSupplier = rotationSupplier;

		addRequirements(swerve);
	}

    /**
     * collects desired offset position
     * @param TagID
     * @return pose of the desired translational offset and rotation
     */
	public Translation2d getAlignmentOffset(int TagID) {
		// TODO: find offsets
		switch (TagID) {
            // Coral Station
            case 1:
            case 2:
			case 12: 
			case 13:
				return new Translation2d(-1,0);

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
				return new Translation2d(-1+0.885815,0+2.8);

            // Friendly Alliance Barge
            case 5:
            case 14:
                return new Translation2d(0,0);

            // Opposing Alliance Barge
            case 4:
            case 15:
            return new Translation2d(0,0);

            // No Tag Found
			default:
				return Translation2d.kZero; // empty pose
		}
	}

	public double getHeading(int TagID) {
		// TODO: find offsets
		switch (TagID) {
            // Coral Station
            case 1:
            case 2:
			case 12: 
			case 13:
				return 2;

			// Reef
			case 6:
			case 7:
				return 0;
			case 8:
				return 15;
			case 9:
            case 10:
            case 11:
            case 17:
            case 18:
            case 19:
            case 20:
            case 21:
            case 22:
				return 0;

            // Friendly Alliance Barge
            case 5:
            case 14:
                return 0;

            // Opposing Alliance Barge
            case 4:
            case 15:
            return 0;

            // No Tag Found
			default:
				return 0; // empty pose
		}
	}

	@Override
	public void execute() {
		// if there is no valid tag, then default driver behaivor is given
		if (LimelightHelpers.getTV(highLimelight) && 
			(LimelightHelpers.getFiducialID(highLimelight) == 1 // HP Station
			|| LimelightHelpers.getFiducialID(highLimelight) == 2
			|| LimelightHelpers.getFiducialID(highLimelight) == 12
			|| LimelightHelpers.getFiducialID(highLimelight) == 13
			|| LimelightHelpers.getFiducialID(highLimelight) == 5 // Barge
			|| LimelightHelpers.getFiducialID(highLimelight) == 14
			|| LimelightHelpers.getFiducialID(highLimelight) == 3 // Barge
			|| LimelightHelpers.getFiducialID(highLimelight) == 16)){
				activeLimelight = highLimelight;
		} else if (LimelightHelpers.getTV(lowLimelight) && 
			(LimelightHelpers.getFiducialID(lowLimelight) == 6 // Reef
			|| LimelightHelpers.getFiducialID(lowLimelight) == 7
			|| LimelightHelpers.getFiducialID(lowLimelight) == 8
			|| LimelightHelpers.getFiducialID(lowLimelight) == 9
			|| LimelightHelpers.getFiducialID(lowLimelight) == 10
			|| LimelightHelpers.getFiducialID(lowLimelight) == 11
			|| LimelightHelpers.getFiducialID(lowLimelight) == 17
			|| LimelightHelpers.getFiducialID(lowLimelight) == 18
			|| LimelightHelpers.getFiducialID(lowLimelight) == 19
			|| LimelightHelpers.getFiducialID(lowLimelight) == 20
			|| LimelightHelpers.getFiducialID(lowLimelight) == 21
			|| LimelightHelpers.getFiducialID(lowLimelight) == 22)) {
				activeLimelight = lowLimelight;
		}else {
			SmartDashboard.putBoolean("Autonomous Lock On", false);
			swerve.drive(
				new Translation2d(
					xSupplier.getAsDouble(), 
					ySupplier.getAsDouble()).times(Constants.Chassis.kMaxSpeed).times(Constants.Operator.kPrecisionMovementMultiplier),
				rotationSupplier.getAsDouble() * Constants.Chassis.kMaxAngularVelocity * 0.5, // Reduce rotation for precision
				true,
				false);
			return;
		}




		// targetTagID = (int) LimelightHelpers.getFiducialID(activeLimelight);
		// SmartDashboard.putBoolean("Autonomous Lock On", true);
		
		// SmartDashboard.putNumber("Targeted April Tag", targetTagID);

		// // The desired translation
		// Pose3d TagToRobotPose = LimelightHelpers.getBotPose3d_TargetSpace(activeLimelight);
		// Translation2d offsetPose = getAlignmentOffset(targetTagID);
		// // Conditionally add offset for a left or a right alternate
		// // if (Math.abs(ySupplier.getAsDouble()) > 0.65) 
		// // 	offsetPose = new Translation2d(
		// // 		offsetPose.getX(), 
		// // 		offsetPose.getY() + (Math.abs(ySupplier.getAsDouble()))/ySupplier.getAsDouble());

		// RobotToTag = new Translation2d(TagToRobotPose.getX(),TagToRobotPose.getY());
		// outputTranslation = RobotToTag.minus(offsetPose);


		// SmartDashboard.putNumber("OutputMoveX", outputTranslation.getX());
		// SmartDashboard.putNumber("OutputMoveY", outputTranslation.getY());
		// SmartDashboard.putNumber("RobotToTagx", RobotToTag.getX());
		// SmartDashboard.putNumber("RobotToTagy", RobotToTag.getY());
		// if (offsetPose.equals(Translation2d.kZero)) return; // checks if LL has valid tag
		// // if (!offsetPose.equals(Translation2d.kZero)) defaultOffsetBehaivior(outputTranslation);	
		// else 
		// if valid tag but no translation, sets desired rotation with operator
		// movement, otherwise full operator control
		swerve.drive(
				new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).times(Constants.Chassis.kMaxSpeed),
				rotationSupplier.getAsDouble() * Constants.Chassis.kMaxAngularVelocity * 0.5,
				true,
				false);
	}

	public void defaultOffsetBehaivior(Translation2d offsetPose){
		// swerve.setTargetHeading(offsetPose.getRotation().getDegrees());

			// possible wait for target heading to bpre near reached if needed
			swerve.drive(
				new Translation2d(
					MathUtil.clamp(outputTranslation.getX(), -Constants.Chassis.kMaxSpeed, Constants.Chassis.kMaxSpeed),
					MathUtil.clamp(outputTranslation.getY(), -Constants.Chassis.kMaxSpeed, Constants.Chassis.kMaxSpeed)),
				0.0,
				false, // check for issues with rotation & field relative
				false);
	}

	@Override
	public boolean isFinished() {
		return MathUtil.isNear(0, outputTranslation.getX(), Constants.Vision.kAlignmentTranslationTolerance) &&
			MathUtil.isNear(0, outputTranslation.getY(), Constants.Vision.kAlignmentTranslationTolerance);
	}

	@Override
	public void end(boolean wasInterrupted) {
		SmartDashboard.putBoolean("Autonomous Lock On", false);
		swerve.drive(new Translation2d(0, 0), 0, true, false);
		swerve.setTargetHeading(swerve.getYaw().getDegrees());
		if (DriverStation.isAutonomous()) {
			swerve.stopModules();
		}
	}
}