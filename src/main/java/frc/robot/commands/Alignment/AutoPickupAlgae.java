package frc.robot.commands.alignment;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakingState;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AutoPickupAlgae extends Command{

    private SwerveDrive swerve;
    private Intake intake;

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier rotSupplier;

    public AutoPickupAlgae(SwerveDrive swerve, Intake intake, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier){
        this.swerve = swerve;
        this.intake = intake;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(Constants.Vision.rightLimelight, Constants.Vision.Pipeline.NeuralNetwork);
        intake.setState(IntakingState.ALGAE_INTAKE);
    }

    @Override
    public void execute() {
        double closeupOffset = 10 * LimelightHelpers.getTA(Constants.Vision.rightLimelight)/20;
        if (LimelightHelpers.getTV(Constants.Vision.rightLimelight))
            swerve.setChassisSpeeds(new ChassisSpeeds(
                xSupplier.getAsDouble() * Constants.Chassis.kMaxSpeed,
                ySupplier.getAsDouble() * Constants.Chassis.kMaxSpeed,
                (Math.abs(LimelightHelpers.getTX(Constants.Vision.rightLimelight)) > 1
                    ?MathUtil.clamp((-LimelightHelpers.getTX(Constants.Vision.rightLimelight)-closeupOffset)/(Constants.Vision.kxFOV/2) * 0.55,
                        -Constants.Chassis.kMaxAngularVelocity * 0.45, Constants.Chassis.kMaxAngularVelocity * 0.7)
                    : 0.0)));
        else swerve.drive(
            new Translation2d(
			        xSupplier.getAsDouble(), 
			        xSupplier.getAsDouble()
                ).times(Constants.Chassis.kMaxAngularVelocity * 0.5), 
                rotSupplier.getAsDouble() * 0.3 * Constants.Chassis.kMaxAngularVelocity,
				true, false);
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex(Constants.Vision.rightLimelight, Constants.Vision.Pipeline.AprilTags);
        if (!intake.getIntakingState().equals(IntakingState.ALGAE_PASSIVE)) intake.stop();
        swerve.drive(new Translation2d(), 0, true, false);
    }

    @Override
    public boolean isFinished() {
        return intake.getIntakingState().equals(IntakingState.ALGAE_PASSIVE);
    }
}
