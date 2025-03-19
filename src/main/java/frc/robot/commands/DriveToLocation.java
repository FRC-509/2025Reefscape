package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.BezierPathGeneration.FieldPosition;
import frc.robot.commands.BezierPathGeneration.Location;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.ThinNT;

public class DriveToLocation extends Command {

    private final SwerveDrive swerve;
    private Pose2d desiredLocation;
    private Translation2d move;

    public DriveToLocation(SwerveDrive swerve){
        this.swerve = swerve;
        this.desiredLocation = Location.Reef_Close_Left.pose2d;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        move = swerve.getEstimatedPose().relativeTo(desiredLocation).getTranslation();
    }

    @Override
    public void execute() {
        move = swerve.getEstimatedPose().relativeTo(desiredLocation).getTranslation();
        ThinNT.putNumber("Move X", move.getX());
        ThinNT.putNumber("Move y", move.getY());
        swerve.setTargetHeading(desiredLocation.getRotation().getDegrees());
        swerve.drive(
			new Translation2d(
				MathUtil.clamp(move.getX(), -Constants.Chassis.kMaxSpeed * Constants.Operator.kPrecisionMovementMultiplier, Constants.Chassis.kMaxSpeed * Constants.Operator.kPrecisionMovementMultiplier),
				MathUtil.clamp(move.getY(), -Constants.Chassis.kMaxSpeed * Constants.Operator.kPrecisionMovementMultiplier, Constants.Chassis.kMaxSpeed * Constants.Operator.kPrecisionMovementMultiplier)),
            // MathUtil.clamp(move.getAngle().getRadians(), -Constants.Chassis.kMaxAngularVelocity * Constants.Operator.kPrecisionMovementMultiplier, Constants.Chassis.kMaxAngularVelocity * Constants.Operator.kPrecisionMovementMultiplier),
			0,
            true,
			false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(move.getX()) < 0.01 && Math.abs(move.getY()) < 0.01;
    }


}
