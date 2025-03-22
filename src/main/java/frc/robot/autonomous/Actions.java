package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.StagingManager;
import frc.robot.commands.StagingManager.StagingState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakingState;
import frc.robot.subsystems.drive.SwerveDrive;

public class Actions {
    
    // public SequentialCommandGroup DriveToAndPlaceL4Coral(
    //         String pathName,
    //         double waitExtensionSeconds,
    //         SwerveDrive swerve,
    //         Elevator elevator,
    //         Arm arm,
    //         Intake intake){
    //     PathValidation path = new PathValidation(pathName);
    //     return new SequentialCommandGroup(
    //         path.errorCancel,
    //         Commands.parallel(
    //             Commands.sequence(
    //                 path.pathCommand,
    //                 Commands.runOnce(() -> swerve.stopModules(), swerve))
    //             ),
    //             Commands.sequence(
    //                 new WaitCommand(waitExtensionSeconds),
    //                 StagingManager.PlaceCoral_L4(elevator, arm)),
    //                 Commands.runOnce(() -> intake.outake(true), intake),
    //                 Commands.waitSeconds(Constants.Intake.kCoralOutakeDelay),
    //                 Commands.runOnce(() -> intake.stop(), intake));
    // }
    
    // public SequentialCommandGroup DriveToAndPlaceMidLevelCoral(
    //         String pathName,
    //         double waitExtensionSeconds,
    //         StagingState level,
    //         SwerveDrive swerve,
    //         Elevator elevator,
    //         Arm arm,
    //         Intake intake){
    //     PathValidation path = new PathValidation(pathName);
    //     return new SequentialCommandGroup(
    //         path.errorCancel,
    //         Commands.parallel(
    //             Commands.sequence(
    //                 path.pathCommand,
    //                 Commands.runOnce(() -> swerve.stopModules(), swerve))
    //             ),
    //             Commands.sequence(
    //                 new WaitCommand(waitExtensionSeconds),
    //                 StagingManager.PlaceCoral_Mid(level, elevator, arm)),
    //         Commands.runOnce(() -> intake.outake(true), intake),
    //         Commands.waitSeconds(Constants.Intake.kCoralOutakeDelay),
    //         Commands.runOnce(() -> intake.stop(), intake));
    // }

    public SequentialCommandGroup DriveToAndCoralStation(
            String pathName,
            double waitExtensionSeconds,
            StagingState level,
            SwerveDrive swerve,
            Elevator elevator,
            Arm arm,
            Intake intake){
        PathValidation path = new PathValidation(pathName);
        return new SequentialCommandGroup(
            path.errorCancel,
            Commands.parallel(
                Commands.sequence(
                    path.pathCommand,
                    Commands.runOnce(() -> swerve.stopModules(), swerve))
                ),
                Commands.sequence(
                    new WaitCommand(waitExtensionSeconds),
                    StagingManager.all(StagingState.CORAL_STATION, elevator, arm),
                    Commands.runOnce(() -> intake.setState(IntakingState.CORAL_INTAKE), intake)));
    }

    public SequentialCommandGroup DriveToAndRemoveAlgae(
            String pathName,
            double waitExtensionSeconds,
            StagingState level,
            SwerveDrive swerve,
            Elevator elevator,
            Arm arm,
            Intake intake){
        PathValidation path = new PathValidation(pathName);
        return new SequentialCommandGroup(
            path.errorCancel,
            Commands.parallel(
                Commands.sequence(
                    path.pathCommand,
                    Commands.runOnce(() -> swerve.stopModules(), swerve))
                ),
                Commands.sequence(
                    new WaitCommand(waitExtensionSeconds),
                    StagingManager.all(StagingState.CORAL_STATION, elevator, arm),
                    Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_INTAKE), intake)));
    }
    
    public static SequentialCommandGroup BargeShot(
            SwerveDrive swerve,
            Elevator elevator,
            Arm arm,
            Intake intake){
        return new SequentialCommandGroup(
            StagingManager.L4_Rising(elevator, arm, intake, () -> false),
            Commands.waitSeconds(0.2),
            Commands.parallel(
                Commands.sequence(
                    Commands.waitSeconds(0.6),
                    Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_OUTAKE))    
                ),
                new DefaultDriveCommand(swerve, 0.5 * Constants.Operator.kPrecisionMovementMultiplier, 0.0, 0.0, false)
            ),
            Commands.waitSeconds(2),
            Commands.runOnce(() -> intake.stop()),
            new DefaultDriveCommand(swerve, 0.0, 0.0, 0.0, true),
            StagingManager.allSafe(StagingState.ZEROED, elevator, arm));
    }

    public static class PathValidation {
        public Command pathCommand;
        public Command errorCancel;
        public PathValidation(String pathName){
            try {
                pathCommand = AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
                errorCancel = new WaitUntilCommand(() -> true);
            } catch (Exception e){
                SmartDashboard.putString("Path Loading", pathName + " DOES NOT EXIST/ERRORED");
                pathCommand = new WaitUntilCommand(() -> false);
                errorCancel = new WaitUntilCommand(() -> false);
            }
        }
    }
}
