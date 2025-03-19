package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BezierPathGeneration.FieldPosition;
import frc.robot.commands.BezierPathGeneration.Location;
import frc.robot.subsystems.drive.SwerveDrive;

public class AlignmentManager {

    private class AlignmentTrigger{
        private BooleanSupplier trigger;
        private Command alignmentCommand;
        private boolean last;

        public AlignmentTrigger(BooleanSupplier trigger, Command alignmentCommand){
            this.trigger = trigger;
            this.alignmentCommand = alignmentCommand;
            this.last = false;
        }

        public void onChange(){
            if (trigger.getAsBoolean() && !last) alignmentCommand.schedule();
            last = trigger.getAsBoolean();
        }
    }

    private SwerveDrive swerve;
    private DoubleSupplier alternateSupplier;

    private AlignmentTrigger reefTrigger;
    private AlignmentTrigger coralStationTrigger;
    private AlignmentTrigger bargeTrigger;

    private Location[] reefLocations = {
    }

    public AlignmentManager(
        DoubleSupplier alternateSupplier,
        BooleanSupplier reefSupplier,
        BooleanSupplier coralStationSupplier,
        BooleanSupplier bargeShotSupplier,
        SwerveDrive swerve
    ){
        this.reefTrigger = new AlignmentTrigger(
            bargeShotSupplier, 
            new ReefAlign(swerve));

        this.coralStationTrigger = new AlignmentTrigger(
            bargeShotSupplier, 
            new CoralStationAlign(swerve));

        this.bargeTrigger = new AlignmentTrigger(
            bargeShotSupplier, 
            new BargeShot(swerve));

        this.swerve = swerve;
    }

    public static FieldPosition getEstimatedFieldPosition(SwerveDrive swerve){
        Pose2d estimatedPose = swerve.getEstimatedPose();
        return new FieldPosition(estimatedPose.getX(), estimatedPose.getY());
    }

    public static FieldPosition findClosestPosition(FieldPosition comparisonPosition, FieldPosition... positions){
        double closestDistance = comparisonPosition.distanceTo(positions[0]), distance = 0;
        FieldPosition closestPosition = positions[0];
        for (int i = 1; i < positions.length; i++){
            distance = comparisonPosition.distanceTo(positions[i]);
            if (distance < closestDistance){
                closestDistance = distance;
                closestPosition = positions[i];
            }
        }
        return closestPosition;
    }

    public class CoralStationAlign extends Command{

        private SwerveDrive swerve;
        private Location coralStation;

        public CoralStationAlign(SwerveDrive swerve){
            this.swerve = swerve;

            FieldPosition estimatedPosition = getEstimatedFieldPosition(swerve);
            this.coralStation = findClosestPosition(estimatedPosition, );
        }
    }

    public class BargeShot extends Command{

    }

    public class ReefAlign extends Command{

    }
}
