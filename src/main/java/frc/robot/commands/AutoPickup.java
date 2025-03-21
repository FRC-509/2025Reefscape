package frc.robot.commands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakingState;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Limelight;

public class AutoPickup extends Command {

	private SwerveDrive swerve;
	private Limelight limelight;
	private Intake intake;
	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;
	private DoubleSupplier rotationSupplier;

	private boolean isFinished = false;

	private double lastTX;
	private double lastDistanceToTarget;
	private boolean lostTarget = false;

	AutoPickup(
			SwerveDrive swerve,
			Limelight limelight,
			Intake intake,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier rotationSupplier) {
		this.swerve = swerve;
		this.limelight = limelight;
		this.intake = intake;

		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.rotationSupplier = rotationSupplier;

		addRequirements(swerve, intake);
	}

	@Override
	public void initialize() {
		isFinished = false;
		lostTarget = false;
		lastTX = 0.0;
		lastDistanceToTarget = (Constants.Vision.kCameraHeightFromGround
				/ Math.tan(Math.toRadians(-limelight.getTY() + -Constants.Vision.kIntakeCameraAngleOffset)));

		SmartDashboard.putBoolean("Autonomous Lock On", false);
		limelight.setPipelineIndex(Constants.Vision.Pipeline.NeuralNetwork);
	}

	@Override
	public void execute() {
		SmartDashboard.putBoolean("Autonomous Lock On", limelight.getTV());

		// Finds distance to target and how much to move
		double angleToTarget = -limelight.getTY() - Constants.Vision.kIntakeCameraAngleOffset;
		double distanceToTargetY = (Constants.Vision.kCameraHeightFromGround
				/ Math.tan(Math.toRadians(angleToTarget)));
		double distanceToTargetX = -(distanceToTargetY * Math.sin(Math.toRadians(limelight.getTX())));

		if (!limelight.getTV() || intake.getIntakingState() == IntakingState.ALGAE_PASSIVE) {
			swerve.drive(
					new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).times(Constants.Chassis.kMaxSpeed),
					rotationSupplier.getAsDouble() * Constants.Chassis.kMaxAngularVelocity * Constants.Operator.kPrecisionRotationMultiplier,
					true,
					false);
		} else if (limelight.getTV() && distanceToTargetY < 3.0) {
			double travelDistanceY;
			double useTX;
			if (!lostTarget) {
				if (Math.abs(distanceToTargetY) < Math
						.abs(lastDistanceToTarget + Constants.Vision.kMaxTargetDistanceVariation)) {
					travelDistanceY = distanceToTargetY;
					useTX = limelight.getTX();
				} else {
					travelDistanceY = lastDistanceToTarget;
					useTX = lastTX;
					lostTarget = true;
				}
			} else {
				travelDistanceY = lastDistanceToTarget / 2;
				useTX = lastTX / 2;
				SmartDashboard.putBoolean("Autonomous Lock On", false);
			}

			swerve.drive(
					new Translation2d(
							MathUtil.clamp(travelDistanceY * 2, -Constants.Chassis.kMaxSpeed, Constants.Chassis.kMaxSpeed),
							MathUtil.clamp(distanceToTargetX * 3, -Constants.Chassis.kMaxSpeed, Constants.Chassis.kMaxSpeed)),
					MathUtil.clamp(Math.toRadians(-useTX * 3),
							-Constants.Chassis.kMaxAngularVelocity, Constants.Chassis.kMaxAngularVelocity),
					false,
					false);
		}

		if (distanceToTargetY < 2) {
			if (intake.getIntakingState() == IntakingState.ALGAE_PASSIVE) isFinished = true;
		}
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}

	@Override
	public void end(boolean wasInterrupted) {
		swerve.drive(new Translation2d(0, 0), 0, true, false);
		SmartDashboard.putBoolean("Autonomous Lock On", false);
	}
}