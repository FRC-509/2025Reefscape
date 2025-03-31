package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.autonomous.Actions.PathValidation;
import frc.robot.commands.staging.StagingManager;
import frc.robot.commands.staging.StagingManager.StagingState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakingState;
import frc.robot.subsystems.drive.SwerveDrive;

public class L4Center extends SequentialCommandGroup {
    

    public L4Center(SwerveDrive swerve, Elevator elevator, Arm arm, Intake intake){
        PathValidation p1 = new PathValidation("L4Centerp1");
        PathValidation p2 = new PathValidation("L4Centerp2");
        PathValidation p3 = new PathValidation("L4Centerp3");
        PathValidation p4 = new PathValidation("L4Centerp4");
        PathValidation p5 = new PathValidation("L4Centerp5");
        Pose2d startPose = new Pose2d(7.578, 4.170, Rotation2d.fromDegrees(0));
        addCommands(
            Commands.runOnce(() -> RobotContainer.autoEndIntakingState = IntakingState.ALGAE_PASSIVE),
            swerve.resetOdometryCmd(startPose),
            Commands.runOnce(() -> intake.setState(IntakingState.CORAL_PASSIVE)),
            Commands.parallel(
                p1.pathCommand,
                Commands.sequence(
                    Commands.waitSeconds(0.2),
                    StagingManager.L4_Rising(elevator, arm, intake, () -> false)
                )
            ),
            Commands.waitSeconds(1),
            Commands.parallel(
                p2.pathCommand,
                Commands.sequence(                        
                    StagingManager.L4_Falling(elevator, arm, intake, () -> true),
                    Commands.waitSeconds(0.1),
                    Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_INTAKE), intake),
                    StagingManager.allSafe(StagingState.ALGAE_LOW, elevator, arm)
                )
            ),
            Commands.waitSeconds(0.25),
            Commands.parallel(
                p3.pathCommand,
                Commands.sequence(
                    Commands.waitSeconds(0.45),
                    StagingManager.L4_Rising(elevator, arm, intake, () -> true),
                    Commands.waitSeconds(1.45),
                    Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_OUTAKE), intake)
                )
            ),
            Commands.waitSeconds(0.25),
            Commands.parallel(
                p4.pathCommand,
                Commands.sequence(
                    StagingManager.L4_Falling(elevator, arm, intake, () -> false),
                    Commands.waitSeconds(0.8),
                    StagingManager.allSafe(StagingState.ALGAE_HIGH, elevator, arm),
                    Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_INTAKE), intake)
                )
            ),
            Commands.waitSeconds(0.25),
            Commands.parallel(
                p5.pathCommand,
                Commands.sequence(
                    Commands.waitSeconds(0.45),
                    StagingManager.L4_Rising(elevator, arm, intake, () -> true),
                    Commands.waitSeconds(1.65),
                    Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_OUTAKE), intake)
                )
            ),
            Commands.runOnce(() -> swerve.stopModules(), swerve),
            Commands.waitSeconds(20)
        );
    }
}
