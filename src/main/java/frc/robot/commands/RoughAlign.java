package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;

public class RoughAlign extends Command{
    
    private SwerveDrive swerve;
    private double targetHeading;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;


    public RoughAlign(double heading,DoubleSupplier xSupplier, DoubleSupplier ySupplier, SwerveDrive swerve){
        this.swerve = swerve;
        this.targetHeading = heading;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(
            new Translation2d(
                xSupplier.getAsDouble(), ySupplier.getAsDouble()
            ).times(Constants.Chassis.kMaxSpeed * Constants.Operator.kPrecisionRotationMultiplier), 
            MathUtil.clamp(swerve.getYaw().getDegrees() - targetHeading, Constants.Chassis.kMaxAngularVelocity * -0.1, Constants.Chassis.kMaxAngularVelocity * 0.1), 
            true,
            true);
    }
}
