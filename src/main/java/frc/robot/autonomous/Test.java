package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.Actions.PathValidation;
import frc.robot.subsystems.drive.SwerveDrive;

public class Test extends SequentialCommandGroup {
    public Test(
            String pathName,
            SwerveDrive swerve){
        PathValidation path = new PathValidation(pathName);
        addCommands(
            path.pathCommand,
            Commands.runOnce(() -> swerve.stopModules(), swerve)
        );
    }
}
