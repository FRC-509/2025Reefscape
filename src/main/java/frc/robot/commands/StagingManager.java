package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.staging.ExtendTo;
import frc.robot.commands.staging.RotateTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class StagingManager {

    public static enum StagingState {
        // Defaults                               
        ZEROED(0.2,0.28),

        // Coral
        CORAL_L4(4.209,0.0),
        CORAL_L3(4.209,-0.144),
        CORAL_L2(3.714,-0.144),
        CORAL_L1(3.714,0.28),
        CORAL_GROUND(0.034,-0.170664),
        CORAL_STATION(0.4,0.054),

        // Algae
        ALGAE_HIGH(0.0,0.0),
        ALGAE_LOW(0.0,0.0),
        ALGAE_GROUND(0.0,0.0);

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
            Commands.runOnce(() -> arm.setRotation(state.rotation), elevator),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> elevator.setExtension(state.extension), arm)
        );
    }

    public static SequentialCommandGroup groundPickup(){
        
    }

    public static SequentialCommandGroup zero(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> arm.setRotation(StagingState.CORAL_STATION.rotation), elevator),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() -> elevator.setExtension(StagingState.ZEROED.extension), arm),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() -> arm.setRotation(StagingState.ZEROED.rotation), elevator)
        );
    }
}
