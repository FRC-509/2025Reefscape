package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.commands.staging.ExtendTo;
import frc.robot.commands.staging.RotateTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class StagingManager {

    public static enum StagingState {
        // Defaults                               
        ZEROED(0.4392,1.31),

        // Coral
        CORAL_L4(5.09375,0.82342),
        CORAL_L3(5.218,0.82342),
        CORAL_L2(3.854,0.809804),
        CORAL_L1(2.69402,0.82342),

        ALGAE_GROUND(0.654541,0.83618),
        CORAL_GROUND(0.654541,0.83618),
        
        CORAL_STATION(2.60278,1.0205),

        // Algae
        ALGAE_HIGH(5.218,0.82342),
        ALGAE_LOW(3.854,0.809804);

        public final double extension;
        public final double rotation;

        StagingState(double extension, double rotation){
            this.extension = extension;
            this.rotation = rotation;
        }
    }

    public static SequentialCommandGroup ZeroState(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RotateTo(StagingState.ALGAE_HIGH.rotation, () -> elevator.isInwardsRotationSafe(), arm),
                new ExtendTo(StagingState.ZEROED.extension, () -> arm.isExtensionSafe(), elevator)),
            new RotateTo(StagingState.ZEROED.rotation, () -> elevator.isInwardsRotationSafe(), arm)
        );
    }

    public static ParallelCommandGroup PlaceCoral_L4(Elevator elevator, Arm arm){
        return new ParallelCommandGroup(
            new RotateTo(StagingState.CORAL_L4.rotation, () -> elevator.isInwardsRotationSafe(), arm),
            new ExtendTo(StagingState.CORAL_L4.extension, () -> arm.isExtensionSafe(), elevator)
        );
    }

    public static ParallelCommandGroup PlaceCoral_Mid(StagingState state,Elevator elevator, Arm arm){
        return new ParallelCommandGroup(
            new RotateTo(state.rotation, () -> elevator.isInwardsRotationSafe(), arm),
            new ExtendTo(state.extension, () -> arm.isExtensionSafe(), elevator)
        );
    }

    public static ParallelCommandGroup GroundPickup(Elevator elevator, Arm arm){
        return new ParallelCommandGroup(
            new RotateTo(StagingState.CORAL_GROUND.rotation, () -> elevator.isInwardsRotationSafe(), arm),
            new ExtendTo(StagingState.CORAL_GROUND.extension, () -> arm.isExtensionSafe(), elevator)
        );
    }

    public static ParallelCommandGroup CoralStation(Elevator elevator, Arm arm){
        return new ParallelCommandGroup(
            new RotateTo(StagingState.CORAL_GROUND.rotation, () -> elevator.isInwardsRotationSafe(), arm),
            new ExtendTo(StagingState.CORAL_GROUND.extension, () -> arm.isExtensionSafe(), elevator)
        );
    }

    public static ParallelCommandGroup AlgaePickup(StagingState level, Elevator elevator, Arm arm){
        return new ParallelCommandGroup(
            new RotateTo(level.rotation, () -> elevator.isInwardsRotationSafe(), arm),
            new ExtendTo(level.extension, () -> arm.isExtensionSafe(), elevator)
        );
    }

    public static SequentialCommandGroup all(StagingState state, Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.setExtension(0.39), elevator),
            Commands.runOnce(() -> arm.setRotation(1.108), arm),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> elevator.setExtension(state.extension), elevator),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> arm.setRotation(state.rotation), arm)
        );
    }

    public static SequentialCommandGroup L4_Rising(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.setExtension(0.39), elevator),
            Commands.runOnce(() -> arm.setRotation(1.108), arm),
            Commands.waitSeconds(0.6), // 0.4
            Commands.runOnce(() -> elevator.setExtension(StagingState.CORAL_L4.extension), elevator),
            Commands.waitSeconds(1.2), // 0.4
            Commands.runOnce(() -> arm.setRotation(StagingState.CORAL_L4.rotation), arm)
        );
    }

    public static SequentialCommandGroup L4_Falling(Elevator elevator, Arm arm, Intake intake){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.setExtension(4.3435), elevator),
            Commands.runOnce(() -> intake.setCommandOutake(true), intake),
            Commands.runOnce(() -> intake.l4Outake(), intake),
            Commands.runOnce(() -> arm.setCoast(), arm),
            Commands.waitSeconds(1.6), // 0.4
            Commands.runOnce(() -> arm.setRotation(1.108), elevator),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() -> elevator.setExtension(StagingState.ZEROED.extension), arm),
            Commands.waitSeconds(2),
            Commands.runOnce(() -> arm.setRotation(StagingState.ZEROED.rotation), elevator)
        );
    }

    public static SequentialCommandGroup groundPickup(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> arm.setRotation(1.108), elevator),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> elevator.setExtension(StagingState.ALGAE_GROUND.extension), arm),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> arm.setRotation(StagingState.CORAL_GROUND.rotation), elevator)
        );
    }

    public static SequentialCommandGroup zero(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> arm.setRotation(1.108), arm),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() -> elevator.setExtension(StagingState.ZEROED.extension), elevator),
            Commands.waitSeconds(0.8
            ),
            Commands.runOnce(() -> arm.setRotation(StagingState.ZEROED.rotation), arm)
        );
    }
}
