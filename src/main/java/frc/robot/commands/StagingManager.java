package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.commands.staging.ExtendTo;
import frc.robot.commands.staging.RotateTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class StagingManager {

    private class StagingTrigger {
        BooleanSupplier booleanSupplier;
        boolean last; 
        Runnable onTrue; 
        Runnable onFalse;

        public StagingTrigger(BooleanSupplier booleanSupplier, boolean last, Runnable onTrue, Runnable onFalse){
            this.booleanSupplier = booleanSupplier;
            this.last = last;
            this.onTrue = onTrue;
            this.onFalse = onFalse;
        }
    }

    private final StagingTrigger L4_StagingTrigger;
    private final StagingTrigger L3_StagingTrigger;
    private final StagingTrigger L2_StagingTrigger;
    private final StagingTrigger L1_StagingTrigger;

    private final StagingTrigger coralStation_StagingTrigger;
    
    private final StagingTrigger coralGround_StagingTrigger;
    private final StagingTrigger algaeGround_StagingTrigger; 

    private final DoubleSupplier extensionSupplier;

    private Runnable quedStage;

    public StagingManager(
        Elevator elevator,
        Arm arm,
        Intake intake,
        BooleanSupplier L4_Supplier,
        BooleanSupplier L3_Supplier,
        BooleanSupplier L2_Supplier,
        BooleanSupplier L1_Supplier,
        BooleanSupplier coralStation_Supplier,
        BooleanSupplier coralGround_Supplier,
        BooleanSupplier algaeGround_Supplier,
        DoubleSupplier extensionSupplier
    ){
        this.L4_StagingTrigger = new StagingTrigger(
            L4_Supplier,
            false, 
            () -> L4_Rising(elevator, arm).schedule(), 
            () -> L4_Falling(elevator, arm, intake).schedule());

        this.L3_StagingTrigger = new StagingTrigger(
            L3_Supplier,
            false, 
            () -> all(StagingState.CORAL_L3, elevator, arm).schedule(), 
            () -> zero(elevator, arm).schedule());

        this.L2_StagingTrigger = new StagingTrigger(
            L2_Supplier,
            false, 
            () -> all(StagingState.CORAL_L2, elevator, arm).schedule(), 
            () -> zero(elevator, arm).schedule());

        this.L1_StagingTrigger = new StagingTrigger(
            L1_Supplier,
            false,
            () -> all(StagingState.CORAL_L1, elevator, arm).schedule(), 
            () -> zero(elevator, arm).schedule());

        this.coralStation_StagingTrigger = new StagingTrigger(
            coralStation_Supplier,
            false, 
            () -> all(StagingState.CORAL_STATION, elevator, arm).schedule(), 
            () -> zero(elevator, arm).schedule());

        this.coralGround_StagingTrigger = new StagingTrigger(
            coralGround_Supplier,
            false, 
            () -> all(StagingState.CORAL_GROUND, elevator, arm).schedule(), 
            () -> zero(elevator, arm).schedule());

        this.algaeGround_StagingTrigger = new StagingTrigger(
            algaeGround_Supplier,
            false, 
            () -> all(StagingState.ALGAE_GROUND, elevator, arm).schedule(), 
            () -> zero(elevator, arm).schedule());
        
        this.extensionSupplier = extensionSupplier;
        this.quedStage = null;
    }

    public void update(){
        onChange(L3_StagingTrigger);
        onChange(L2_StagingTrigger);
        onChange(L1_StagingTrigger);

        onChange(coralStation_StagingTrigger);
        onChange(coralGround_StagingTrigger);
        onChange(algaeGround_StagingTrigger);

        onChange(L4_StagingTrigger);

        if (quedStage != null && extensionSupplier.getAsDouble() < StagingManager.StagingState.ALGAE_GROUND.extension){
            quedStage.run();
            quedStage = null;
        }
    }

    void onChange(StagingTrigger trigger) {
        if (!trigger.last && trigger.booleanSupplier.getAsBoolean()){ // on true
            quedStage = trigger.onTrue;
        } else if (trigger.last && !trigger.booleanSupplier.getAsBoolean()) { // on false
            trigger.onFalse.run();
        }
        trigger.last = trigger.booleanSupplier.getAsBoolean(); 
    }


    // static staging states

    public static final double kRotationSafeExtension = 0.1579;
    public static final double kGroundRotation = 0.432617;
    public static final double kExtensionSafeRotation = 0.20148;

    public static enum StagingState {
        // Defaults                               
        ZEROED(0.111523,0.0),
        SAFE(0.061523,0.247803),

        // Coral
        CORAL_L4(4.777822,0.1440143),
        CORAL_L3(4.828,0.48338),
        CORAL_L2(3.387207,0.48338),
        CORAL_L1(1.8645,0.426),

        ALGAE_GROUND(0.3322,0.474609),
        CORAL_GROUND(0.3828,0.462402),
        
        CORAL_STATION(1.938,0.304195),

        // Algae
        ALGAE_HIGH(3.387207,0.48338),
        ALGAE_LOW(3.387207,0.48338);

        public final double extension;
        public final double rotation;

        StagingState(double extension, double rotation){
            this.extension = extension;
            this.rotation = rotation;
        }
    }

    public static SequentialCommandGroup all(StagingState state, Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.setExtension(StagingState.SAFE.extension), elevator),
            Commands.runOnce(() -> arm.setRotation(StagingState.SAFE.rotation), arm),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> elevator.setExtension(state.extension), elevator),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> arm.setRotation(state.rotation), arm)
        );
    }

    public static SequentialCommandGroup L4_Rising(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.setExtension(StagingState.SAFE.extension), elevator),
            Commands.runOnce(() -> arm.setRotation(StagingState.SAFE.rotation), arm),
            Commands.waitSeconds(0.6), // 0.4
            Commands.runOnce(() -> elevator.setExtension(StagingState.CORAL_L4.extension), elevator),
            Commands.waitSeconds(1.2), // 0.4
            Commands.runOnce(() -> arm.setRotation(StagingState.CORAL_L4.rotation), arm)
        );
    }

    public static SequentialCommandGroup L4_Falling(Elevator elevator, Arm arm, Intake intake){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.setExtension(4), elevator),
            Commands.runOnce(() -> intake.setCommandOutake(true), intake),
            Commands.runOnce(() -> intake.L4Outake(), intake),
            Commands.runOnce(() -> arm.setCoast(), arm),
            Commands.waitSeconds(1.6), // 0.4
            Commands.runOnce(() -> arm.setRotation(StagingState.SAFE.rotation), elevator),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() -> elevator.setExtension(StagingState.ZEROED.extension), arm),
            Commands.waitSeconds(2),
            Commands.runOnce(() -> arm.setRotation(StagingState.ZEROED.rotation), elevator)
        );
    }

    public static SequentialCommandGroup groundPickup(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> arm.setRotation(StagingState.SAFE.rotation), elevator),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> elevator.setExtension(StagingState.ALGAE_GROUND.extension), arm),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> arm.setRotation(StagingState.CORAL_GROUND.rotation), elevator)
        );
    }

    public static SequentialCommandGroup zero(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> arm.setRotation(StagingState.SAFE.rotation), arm),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() -> elevator.setExtension(StagingState.ZEROED.extension), elevator),
            Commands.waitSeconds(0.8),
            Commands.runOnce(() -> arm.setRotation(StagingState.ZEROED.rotation), arm)
        );
    }

    public static ParallelCommandGroup allCC(StagingState state, Elevator elevator, Arm arm){
        return new ParallelCommandGroup(
            new RotateTo(state.rotation, () -> elevator.isInwardsRotationSafe(), arm),
            new ExtendTo(state.extension, () -> arm.isExtensionSafe(), elevator)
        );
    }
}
