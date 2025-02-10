package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.staging.ExtendTo;
import frc.robot.commands.staging.RotateTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class StagingManager {

    public static enum StagingState {
        // Defaults                               
        ZEROED(0.0,0.0),

        // Coral
        CORAL_L4(0.0,0.0),
        CORAL_L3(0.0,0.0),
        CORAL_L2(0.0,0.0),
        CORAL_L1(0.0,0.0),
        CORAL_GROUND(0.0,0.0),
        CORAL_STATION(0.0,0.0),

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

    public static ParallelCommandGroup CarryingPosition(Elevator elevator, Arm arm){
        return new ParallelCommandGroup(
            new RotateTo(StagingState.CORAL_STATION.rotation, () -> elevator.isInwardsRotationSafe(), arm),
            new ExtendTo(StagingState.CORAL_STATION.extension, () -> arm.isExtensionSafe(), elevator)
        );
    }
}
