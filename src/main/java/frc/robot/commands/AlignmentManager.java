package frc.robot.commands;

import java.lang.constant.Constable;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.autonomous.Actions;
import frc.robot.commands.BezierPathGeneration.FieldPosition;
import frc.robot.commands.BezierPathGeneration.Location;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
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

    // private Location[] reefLocations = {
    // }

    public AlignmentManager(
        DoubleSupplier alternateSupplier,
        BooleanSupplier reefSupplier,
        BooleanSupplier coralStationSupplier,
        BooleanSupplier bargeShotSupplier,
        SwerveDrive swerve,
        Elevator elevator,
        Arm arm,
        Intake intake
    ){
        this.reefTrigger = new AlignmentTrigger(
            bargeShotSupplier, 
            Commands.none());

        this.coralStationTrigger = new AlignmentTrigger(
            bargeShotSupplier, 
            new CoralStationAlign(swerve));

        this.bargeTrigger = new AlignmentTrigger(
            bargeShotSupplier, 
            new BargeShot(swerve, elevator, arm, intake, alternateSupplier));

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
            // this.coralStation = findClosestPosition(estimatedPosition, );
        }
    }

    public class BargeShot extends Command{

        private SwerveDrive swerve;
        private DoubleSupplier leftStick;
        private boolean aligned = false;
        private double bargeOffsetPosition;
        private static final double lineupOffset = 2.0;
        private FieldPosition estimatedFieldPosition;
        private Command actualShot;

        public BargeShot(SwerveDrive swerveDrive, Elevator elevator, Arm arm, Intake intake, DoubleSupplier leftStick){
            this.swerve = swerveDrive;
            this.leftStick = leftStick;
            this.bargeOffsetPosition = Constants.Field.kFullFieldLength/2;
            this.actualShot = Actions.BargeShot(swerveDrive, elevator, arm, intake);
            addRequirements(swerveDrive);
        }

        @Override
        public void initialize() {
            bargeOffsetPosition += SwerveDrive.getAlliance().equals(Alliance.Red) ? lineupOffset : -lineupOffset;
            estimatedFieldPosition = getEstimatedFieldPosition(swerve);
        }

        @Override
        public void execute() {
            estimatedFieldPosition = getEstimatedFieldPosition(swerve);
            swerve.setTargetHeading(0);
            swerve.drive(
                new Translation2d(
                    MathUtil.clamp(
                        Math.abs(estimatedFieldPosition.x - bargeOffsetPosition),
                        -Constants.Chassis.kMaxSpeed,Constants.Chassis.kMaxSpeed),
                    leftStick.getAsDouble() * Constants.Chassis.kMaxSpeed * Constants.Operator.kPrecisionMovementMultiplier
                ), 0, true, false);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(estimatedFieldPosition.x - bargeOffsetPosition) < 0.075;
        }

        @Override
        public void end(boolean interrupted) {
            swerve.stopModules();
            if (!interrupted) actualShot.schedule();
        }
    }

    public class ReefAlign extends Command{

    }
}
