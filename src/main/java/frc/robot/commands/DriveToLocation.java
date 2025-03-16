package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.BezierPathGeneration.FieldPosition;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.ThinNT;

public class DriveToLocation extends Command {
    
    public enum Location {
        
        Reef_Close_Center(14.5, 4, 0.0),
        Reef_Close_Left(14.5, 3.87, 0.0), // find
        Reef_Close_Right(14.5, 4.1, 0.0), // find

        Reef_CloseRight_Center(0,0,-60), // find
        Reef_CloseRight_Left(0,0,-60), // find
        Reef_CloseRight_Right(0,0,0), // find

        Reef_CloseLeft_Center(0,0,0), // find
        Reef_CloseLeft_Left(0,0,0), // find
        Reef_CloseLeft_Right(0,0,0), // find

        Reef_FarRight_Center(0,0,0), // find
        Reef_FarRight_Left(0,0,0), // find
        Reef_FarRight_Right(0,0,0), // find

        Reef_FarLeft_Center(0,0,0), // find
        Reef_FarLeft_Left(0,0,0), // find
        Reef_FarLeft_Right(0,0,0), // find
        
        Reef_Far_Center(0,0,0), // find
        Reef_Far_Left(0,0,0), // find
        Reef_Far_Right(0,0,0), // find

        CoralStation_Left(0,0,0),
        CoralStation_Right(0,0,0),
        
        BargeShot(0,0,0);

        public final FieldPosition position;
        public final Rotation2d rotation;
        public final Pose2d pose2d;
        /**
         * @param x the x coordinate on the field
         * @param y the y coordinate on the field
         * @param headingDegrees the desired final target heading, in degrees, from 180 to -180 
         */
        Location(double x, double y, double headingDegrees){
            this.position = new FieldPosition(x, y);
            this.rotation = Rotation2d.fromDegrees(headingDegrees);
            this.pose2d = new Pose2d(x,y,rotation);
        }
    }

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
