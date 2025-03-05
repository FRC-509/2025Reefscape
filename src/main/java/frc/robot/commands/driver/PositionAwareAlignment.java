package frc.robot.commands.driver;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDrive;

public class PositionAwareAlignment extends Command {
    
    private final SwerveDrive swerve;
    private final TargetPosition targetPosition;

    public static enum TargetPosition{
        CLOSE_REEF,
        FAR_REEF,
        CLOSE_LEFT_REEF,
        CLOSE_RIGHT_REEF,
        FAR_LEFT_REEF,
        FAR_RIGHT_REEF;
    }

    public PositionAwareAlignment(TargetPosition targetPosition, SwerveDrive swerve){
        this.targetPosition = targetPosition;
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            PathConstraints.unlimitedConstraints(12.0),
            null,
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // set holonomic rotation here
        );
        path.preventFlipping = true;
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
