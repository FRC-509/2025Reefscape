package frc.robot.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autonomous.Actions.PathValidation;
import frc.robot.subsystems.drive.SwerveDrive;

public class Test extends SequentialCommandGroup {
    public Test(
            SwerveDrive swerve){
        addCommands(
            Commands.runOnce(() ->swerve.setChassisSpeeds(new ChassisSpeeds(Constants.Chassis.kMaxSpeed * 0.1, 0, 0)),swerve),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() ->swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0)),swerve)
        );
    }
}
