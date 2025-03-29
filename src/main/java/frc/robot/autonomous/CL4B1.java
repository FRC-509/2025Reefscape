package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.autonomous.Actions.PathValidation;
import frc.robot.commands.staging.StagingManager;
import frc.robot.commands.staging.StagingManager.StagingState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakingState;
import frc.robot.subsystems.drive.SwerveDrive;

public class CL4B1 extends SequentialCommandGroup {
    public CL4B1(
            SwerveDrive swerve, Elevator elevator, Arm arm, Intake intake){
        PathValidation p1 = new PathValidation("CL4B1p1");
        PathValidation p2 = new PathValidation("CL4B1p2");
        PathValidation p3 = new PathValidation("CL4B1p3");
        Pose2d startPose = new Pose2d(7.558, 4.050, Rotation2d.fromDegrees(0));
        addCommands(
            Commands.runOnce(() -> RobotContainer.autoEndIntakingState = IntakingState.STOP),
            swerve.resetOdometryCmd(startPose),
            Commands.runOnce(() -> intake.setState(IntakingState.CORAL_PASSIVE)),
            Commands.parallel(
                p1.pathCommand,
                Commands.sequence(
                    StagingManager.L4_Rising(elevator, arm, intake, () -> false),
                    Commands.waitSeconds(4)
                )
            ),
            Commands.parallel(
                Commands.sequence(
                    StagingManager.L4_Falling(elevator, arm, intake, () -> true),
                    Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_INTAKE), intake),
                    StagingManager.allSafe(StagingState.ALGAE_HIGH, elevator, arm)
                ),
                p2.pathCommand
            ),
            Commands.waitSeconds(0.3),
            Commands.parallel(
                p3.pathCommand,
                Commands.sequence(
                    Commands.waitSeconds(0.65),
                    StagingManager.L4_Rising(elevator, arm, intake, () -> true),
                    Commands.waitSeconds(0.35),
                    Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_OUTAKE), intake)
                )
            ),
            StagingManager.zero(elevator, arm, intake),
            Commands.runOnce(() -> swerve.stopModules(), swerve),
            Commands.waitSeconds(20)
        );
    }
}
