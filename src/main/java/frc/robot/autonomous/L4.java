package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.autonomous.Actions.PathValidation;
import frc.robot.commands.staging.StagingManager;
import frc.robot.commands.staging.StagingManager.StagingState;
import frc.robot.subsystems.Intake.IntakingState;
import frc.robot.subsystems.drive.SwerveDrive;

public class L4 extends SequentialCommandGroup {
    public L4(
            SwerveDrive swerve, Elevator elevator, Arm arm, Intake intake){
        PathValidation p1 = new PathValidation("L4p1");
        PathValidation p2 = new PathValidation("L4p2");
        PathValidation p3 = new PathValidation("L4p3");
        Pose2d startPose = new Pose2d(7.568, 6.114, Rotation2d.fromDegrees(0));
        addCommands(
            swerve.resetOdometryCmd(startPose),
            Commands.runOnce(() -> intake.setState(IntakingState.CORAL_PASSIVE)),
            Commands.parallel(
                p1.pathCommand,
                Commands.sequence(
                    Commands.waitSeconds(1.6),
                    StagingManager.L4_Rising(elevator, arm, intake, () -> false)
                )
            ),
            Commands.waitSeconds(4),
            Commands.deadline(
                StagingManager.L4_Falling(elevator, arm, intake, () -> false),
                Commands.runOnce(() -> swerve.setChassisSpeeds(new ChassisSpeeds(-0.2, 0, 0)), swerve)
            ),
            Commands.runOnce(() -> swerve.stopModules(), swerve),
            Commands.waitSeconds(20)
        );
    }
}
