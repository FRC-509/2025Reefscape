package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.drive.SwerveDrive;

public class Leave extends SequentialCommandGroup {
	public Leave(double percentSpeed, double wait, SwerveDrive swerve) {
        addCommands(Commands.sequence(
            new DefaultDriveCommand(
                swerve, 
                percentSpeed, 
                0.0d,
			    0.0d,
                false),
            Commands.waitSeconds(wait),
            Commands.runOnce(() -> swerve.stopModules(), swerve)));
	}
}