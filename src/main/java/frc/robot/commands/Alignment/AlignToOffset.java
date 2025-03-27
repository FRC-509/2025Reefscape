package frc.robot.commands.alignment;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AlignToOffset extends Command{

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

    public enum Tag{
        REEF(0.3048),
        CORAL_STATION(1.4986),
        BARGE(2);

        double targetHeight; // meters
        Tag(double targetHeight){ this.targetHeight = targetHeight; }
    }

    private Limelight limelight;
    private SwerveDrive swerve;
    private double offsetAway;
    private double offsetHorizontal;
    private double desiredHeading;
    private Tag tag;

    public AlignToOffset(
        Limelight limelight, 
        SwerveDrive swerve, 
        double offsetAway, 
        double offsetHorizontal,
        double desiredHeading){
        this.limelight = limelight;
        this.swerve = swerve;
        this.offsetAway = offsetAway;
        this.offsetHorizontal = offsetHorizontal;
        this.desiredHeading = desiredHeading;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (MathUtil.isNear(desiredHeading, swerve.getYaw().getDegrees(), 0.75) || LimelightHelpers.getTV(limelight.name))
            swerve.drive(
                new Translation2d(
                    MathUtil.clamp((getXDistance(Tag.REEF.targetHeight) - offsetAway) * 0.2, -Constants.Chassis.kMaxSpeed, Constants.Chassis.kMaxSpeed),
                    MathUtil.clamp(0.0, -Constants.Chassis.kMaxSpeed, Constants.Chassis.kMaxSpeed)
                ),
                // MathUtil.clamp(Math.toRadians(desiredHeading-swerve.getYaw().getDegrees()), 
                //     -Constants.Chassis.kMaxAngularVelocity, Constants.Chassis.kMaxAngularVelocity), 
                0,
                false,
                true);
        else swerve.drive(new Translation2d(), desiredHeading, true, false);
        SmartDashboard.putNumber("X distance",getXDistance(Tag.REEF.targetHeight));
        SmartDashboard.putNumber("y distance",getYDistance(Tag.REEF.targetHeight));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Forwards, away from the robot
     */
    public double getXDistance(double targetHeight){
        return (targetHeight - limelight.mountHeight) 
            / Math.tan(Math.toRadians(limelight.verticalAngle + LimelightHelpers.getTY(limelight.name)));
    }

    /** 
     * Horizontal, away from the robot
     */
    public double getYDistance(double targetHeight){
        return (getXDistance(targetHeight) - limelight.mountDistanceFromCenter) 
            / Math.tan(Math.toRadians(limelight.horizontalAngle - LimelightHelpers.getTX(limelight.name)));
    }
}
