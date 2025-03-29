package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.autonomous.Actions.PathValidation;
import frc.robot.commands.staging.StagingManager;
import frc.robot.commands.staging.StagingManager.StagingState;
import frc.robot.subsystems.Intake.IntakingState;
import frc.robot.subsystems.drive.SwerveDrive;

public class B3L1 extends SequentialCommandGroup {
    public B3L1(
            SwerveDrive swerve, Elevator elevator, Arm arm, Intake intake){
        PathValidation p1 = new PathValidation("B3L1p1");
        PathValidation p2 = new PathValidation("B3L1p2");
        PathValidation p3 = new PathValidation("B3L1p3");
        PathValidation p4 = new PathValidation("B3L1p4");
        PathValidation p5 = new PathValidation("B3L1p5");
        PathValidation p6 = new PathValidation("B3L1p6");
        Pose2d startPose = new Pose2d(7.568, 6.114, Rotation2d.fromDegrees(0));
        addCommands(
            Commands.runOnce(() -> RobotContainer.autoEndIntakingState = IntakingState.ALGAE_PASSIVE),
            swerve.resetOdometryCmd(startPose),
            Commands.runOnce(() -> intake.setState(IntakingState.CORAL_PASSIVE)),
            Commands.parallel(
                p1.pathCommand,
                Commands.sequence(
                    Commands.waitSeconds(0.6),
                    StagingManager.allSafe(StagingState.CORAL_L1, elevator, arm),
                    Commands.waitSeconds(1.5),
                    Commands.runOnce(() -> intake.setState(IntakingState.CORAL_OUTAKE), intake)
                )
            ),
            Commands.waitSeconds(Constants.Intake.kCoralOutakeDelay/3.5),
            Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_INTAKE), intake),
            Commands.parallel(
                p2.pathCommand,
                Commands.sequence(
                    Commands.waitSeconds(0.3),
                    StagingManager.allSafe(StagingState.ALGAE_HIGH, elevator, arm)
                )
            ),
            Commands.waitSeconds(0.4),
            Commands.parallel(
                p3.pathCommand,
                Commands.sequence(
                    Commands.waitSeconds(0.65),
                    StagingManager.L4_Rising(elevator, arm, intake, () -> true),
                    Commands.waitSeconds(0.45),
                    Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_OUTAKE), intake)
                )
            ),
            StagingManager.L4_Falling(elevator, arm, intake, () -> false),
            Commands.parallel(
                p4.pathCommand,
                Commands.sequence(
                    Commands.waitSeconds(1),
                    StagingManager.allSafe(StagingState.ALGAE_LOW, elevator, arm),
                    Commands.waitSeconds(0.6),
                    Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_INTAKE), intake)
                )
            ),
            Commands.waitSeconds(0.2),
            Commands.parallel(
                p5.pathCommand,
                Commands.sequence(
                    Commands.waitSeconds(1.6),
                    StagingManager.L4_Rising(elevator, arm, intake, () -> true),
                    Commands.waitSeconds(0.6),
                    Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_OUTAKE), intake)
                )
            ),
            StagingManager.L4_Falling(elevator, arm, intake, () -> false),
            Commands.waitSeconds(0.2),
            p6.pathCommand,
            Commands.runOnce(() -> swerve.stopModules(), swerve),
            Commands.waitSeconds(20)
        );
    }
}
