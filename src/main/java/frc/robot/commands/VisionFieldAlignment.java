package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.LimelightHelpers;

import java.util.function.DoubleSupplier;


public class VisionFieldAlignment extends Command {

	private SwerveDrive swerve;
	private String highLimelight;
	private String lowLimelight;

	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;
	private DoubleSupplier rotationSupplier;

	private int targetTagID;

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

	public double getHeading(int TagID) {
		switch (TagID) {
            // Coral Station
            case 1:
			case 13:
			case 9:
			case 22:
				return -137;

			case 2:
			case 12:
			case 11:
			case 20:
				return 137;

			case 6:
			case 19:
				return 47;

			case 8:
			case 17:
				return 47;

			case 7:
			case 18:
				return 0;

			case 10:
			case 21:
				return 180;

			default:
				return 0;
		}
	}

	@Override
	public void execute() {
		SmartDashboard.putBoolean("Autonomous Lock On", true);
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

				double heading = getHeading(targetTagID);
				swerve.setTargetHeading(heading);
				if (MathUtil.isNear(heading, swerve.getYaw().getDegrees(), 0.75)) swerve.drive(
					new Translation2d(
						LimelightHelpers.getTY(highLimelight)/(Constants.Vision.kyFOV),
						LimelightHelpers.getTX(highLimelight)/(Constants.Vision.kxFOV)
					).times(Constants.Chassis.kMaxSpeed * 0.8), 
					0, 
					false, 
					false);

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

				double heading = getHeading(targetTagID);
				swerve.setTargetHeading(heading);
				if (MathUtil.isNear(heading, swerve.getYaw().getDegrees(), 0.75)) swerve.drive(
					new Translation2d(
						LimelightHelpers.getTY(highLimelight)/(Constants.Vision.kyFOV),
						LimelightHelpers.getTX(highLimelight)/(Constants.Vision.kxFOV)
					).times(Constants.Chassis.kMaxSpeed * 0.8), 
					0, 
					false, 
					false);

		}else {
			SmartDashboard.putBoolean("Autonomous Lock On", false);
			swerve.drive(
				new Translation2d(
					xSupplier.getAsDouble(), 
					ySupplier.getAsDouble()
				).times(Constants.Chassis.kMaxSpeed * Constants.Operator.kPrecisionMovementMultiplier),
				rotationSupplier.getAsDouble() * Constants.Chassis.kMaxAngularVelocity * 0.5, // Reduce rotation for precision
				true,
				false);
		}
	}

	@Override
	public void end(boolean wasInterrupted) {
		SmartDashboard.putBoolean("Autonomous Lock On", false);
		swerve.drive(new Translation2d(0, 0), 0, true, false);
		swerve.setTargetHeading(swerve.getYaw().getDegrees());
		if (DriverStation.isAutonomous()) swerve.stopModules();
	}
}