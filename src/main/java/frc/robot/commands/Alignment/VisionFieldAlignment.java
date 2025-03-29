package frc.robot.commands.alignment;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.LimelightHelpers;

import java.util.function.DoubleSupplier;


public class VisionFieldAlignment extends Command {

	public static class Limelight{
        String name;
        double verticalAngle; // how many degrees back is your limelight rotated
        double horizontalAngle; // how many degrees to the right is your limelight rotated
        double mountHeight;
        double mountDistanceFromCenter;
        public Limelight(String name, double mountHeight, double mountDistanceFromCenter, double verticalAngle, double horizontalAngle){
            this.name = name; 
            this.verticalAngle = verticalAngle; 
            this.horizontalAngle = horizontalAngle;
            this.mountHeight = mountHeight;
            this.mountDistanceFromCenter = mountDistanceFromCenter;
        }
    }

	private SwerveDrive swerve;
	private final Limelight highLimelight = new Limelight(
			Constants.Vision.leftLimelight, 
			0, 
			0, 
			0, 
			0);
	private final Limelight lowLimelight = new Limelight(
			Constants.Vision.rightLimelight, 
			0,
			0, 
			0, 
			0);
	private Limelight activeLimelight;

	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;
	private DoubleSupplier rotationSupplier;

	private int lastTagID;
	private Runnable alignmentAction;

	private boolean headingReached;
	private boolean begunHeadingAlignment;
	private Timer headingTimer;

	private static final double kReefAlignedTagPercent = 2.33;
	private static final double kReefAlignedLeftTx = 7.8;
	private static final double kReefAlignedRightTx = -16.16;
	private static final double ktP = 0.5;
	private static final double krP = 0.2;

	// Meant to be an "isDownBind" command
	public VisionFieldAlignment(
			SwerveDrive swerve,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier rotationSupplier) {
		this.swerve = swerve;

		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.rotationSupplier = rotationSupplier;

		this.headingTimer = new Timer();

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		LimelightHelpers.setPipelineIndex(highLimelight.name, Constants.Vision.Pipeline.AprilTags);
		LimelightHelpers.setPipelineIndex(lowLimelight.name, Constants.Vision.Pipeline.AprilTags);
		headingReached = false;
		begunHeadingAlignment = false;
		lastTagID = -1;
		alignmentAction = () -> manualDrive();
	}

	@Override
	public void execute() {
		SmartDashboard.putBoolean("Autonomous Lock On", true);
		if (!LimelightHelpers.getTV(highLimelight.name) && !LimelightHelpers.getTV(lowLimelight.name)) { manualDrive(); return; }

		// If both limelights see a target, prioritize the limelight with the tag taking up the larger percentage camera space 
		if (LimelightHelpers.getTV(highLimelight.name) && LimelightHelpers.getTV(lowLimelight.name))
			activeLimelight = LimelightHelpers.getTA(highLimelight.name) < LimelightHelpers.getTA(lowLimelight.name) 
				? lowLimelight : highLimelight;
		else activeLimelight = LimelightHelpers.getTV(lowLimelight.name) ? lowLimelight : highLimelight; // Otherwise whichever has tag
		
		if (LimelightHelpers.getFiducialID(activeLimelight.name) != lastTagID){
			begunHeadingAlignment = false;
			headingReached = false;
			headingTimer.restart();
			alignmentAction = getBehavior();
		}

		SmartDashboard.putNumber("TA", LimelightHelpers.getTA(activeLimelight.name));
		SmartDashboard.putNumber("Tag ID", LimelightHelpers.getFiducialID(activeLimelight.name));
		SmartDashboard.putNumber("Forwards Move", MathUtil.clamp((kReefAlignedTagPercent - LimelightHelpers.getTA(activeLimelight.name))/kReefAlignedTagPercent * Constants.Chassis.kMaxSpeed * ktP,
			-Constants.Chassis.kMaxSpeed,
			Constants.Chassis.kMaxSpeed));
		SmartDashboard.putString("Active Limelight", activeLimelight.name);
		alignmentAction.run();
	}

	private Runnable getBehavior(){
		switch ((int)LimelightHelpers.getFiducialID(activeLimelight.name)) {
			// Coral Station
			case 1: case 13: return () -> coralStationAlign(137);
			case 2: case 12: return () -> coralStationAlign(-137);

			// Reef
			case 9: case 22: return () -> reefAlign(-137);
			case 11: case 20: return () -> reefAlign(137);
			case 6: case 19: return () -> reefAlign(47);
			case 8: case 17: return () -> reefAlign(-47);
			case 7: case 18: return () -> reefAlign(0);
			case 10: case 21: return () -> reefAlign(180);
			// Barge
			case 5: case 14: return () -> bargeAlign();
			default: 
				return () -> manualDrive();
		}
	}

	public void reefAlign(double heading){
		// if (!begunHeadingAlignment) {
		// 	swerve.setTargetHeading(-90);
		// 	begunHeadingAlignment = true;
		// } else if (!headingReached && (headingTimer.get() > 1 || MathUtil.isNear(-90, swerve.getYaw().getDegrees(), 1))) {
		// 	swerve.setTargetHeading(swerve.getYaw().getDegrees());
		// 	headingReached = true;
		// } else 
		
		double offsetTx;
		if (Math.abs(rotationSupplier.getAsDouble()) < 0.4) offsetTx = 0;
		else offsetTx = rotationSupplier.getAsDouble() < 0 ? kReefAlignedLeftTx : kReefAlignedRightTx;

		swerve.drive(
			new Translation2d(
				MathUtil.clamp((kReefAlignedTagPercent - LimelightHelpers.getTA(activeLimelight.name))/kReefAlignedTagPercent,
					-Constants.Chassis.kMaxSpeed,
					Constants.Chassis.kMaxSpeed),
				-(LimelightHelpers.getTX(activeLimelight.name)+offsetTx)/(Constants.Vision.kxFOV) * 0.5
			).times(Constants.Chassis.kMaxSpeed * ktP), 0, false, false);
	}

	public void coralStationAlign(double heading){
		if (!begunHeadingAlignment) {
			swerve.setTargetHeading(heading);
			begunHeadingAlignment = true;
		} else if (!headingReached && (headingTimer.get() > 1 || MathUtil.isNear(heading, swerve.getYaw().getDegrees(), 1))) {
			swerve.setTargetHeading(swerve.getYaw().getDegrees());
			headingReached = true;
		} else swerve.drive(
			new Translation2d(
				LimelightHelpers.getTY(highLimelight.name)/(Constants.Vision.kyFOV),
				LimelightHelpers.getTX(highLimelight.name)/(Constants.Vision.kxFOV)
			).times(Constants.Chassis.kMaxSpeed * 0.8), 
			0,
			false,
			false);
	}

	public void bargeAlign(){
		if (!begunHeadingAlignment) {
			swerve.setTargetHeading(0);
			begunHeadingAlignment = true;
		} else if (!headingReached && (headingTimer.get() > 1 || MathUtil.isNear(0, swerve.getYaw().getDegrees(), 1))) {
			swerve.setTargetHeading(swerve.getYaw().getDegrees());
			headingReached = true;
		} else swerve.drive(
			new Translation2d(
				LimelightHelpers.getTY(highLimelight.name)/(Constants.Vision.kyFOV),
				LimelightHelpers.getTX(highLimelight.name)/(Constants.Vision.kxFOV)
			).times(Constants.Chassis.kMaxSpeed * 0.8), 
			0, 
			false, 
			false);
	}

	public void manualDrive(){
		swerve.drive(
			new Translation2d(
				xSupplier.getAsDouble(),
				ySupplier.getAsDouble()
			).times(Constants.Chassis.kMaxSpeed * 0.55), 
			0,
			true, 
			false);
	}

	@Override
	public void end(boolean wasInterrupted) {
		SmartDashboard.putBoolean("Autonomous Lock On", false);
		swerve.drive(new Translation2d(0, 0), 0, true, false);
		swerve.setTargetHeading(swerve.getYaw().getDegrees());
		if (DriverStation.isAutonomous()) swerve.stopModules();
	}
}