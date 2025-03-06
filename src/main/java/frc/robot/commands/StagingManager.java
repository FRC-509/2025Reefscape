package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
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
