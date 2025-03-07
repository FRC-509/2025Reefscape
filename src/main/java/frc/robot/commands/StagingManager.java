package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
            () -> zero(elevator, arm, intake).schedule());

        this.L2_StagingTrigger = new StagingTrigger(
            L2_Supplier,
            false, 
            () -> all(StagingState.CORAL_L2, elevator, arm).schedule(), 
            () -> zero(elevator, arm, intake).schedule());

        this.L1_StagingTrigger = new StagingTrigger(
            L1_Supplier,
            false,
            () -> all(StagingState.CORAL_L1, elevator, arm).schedule(), 
            () -> zero(elevator, arm, intake).schedule());

        this.coralStation_StagingTrigger = new StagingTrigger(
            coralStation_Supplier,
            false, 
            () -> all(StagingState.CORAL_STATION, elevator, arm).schedule(), 
            () -> zero(elevator, arm, intake).schedule());

        this.coralGround_StagingTrigger = new StagingTrigger(
            coralGround_Supplier,
            false, 
            () -> all(StagingState.CORAL_GROUND, elevator, arm).schedule(), 
            () -> zero(elevator, arm, intake).schedule());

        this.algaeGround_StagingTrigger = new StagingTrigger(
            algaeGround_Supplier,
            false, 
            () -> all(StagingState.ALGAE_GROUND, elevator, arm).schedule(), 
            () -> zero(elevator, arm, intake).schedule());
        
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
        SAFE(0.061523,0.257803),
        ALGAE_SAFE(0.061523, 0.2233887),

        // Coral
        CORAL_L4(4.777822,0.1440143),
        CORAL_L3(4.628,0.426),
        CORAL_L2(3.387207,0.426),
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

    public static SequentialCommandGroup L4_Rising(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            all(StagingState.SAFE, elevator, arm),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> elevator.setExtension(StagingState.CORAL_L4.extension), elevator),
            Commands.waitSeconds(0.6),
            Commands.runOnce(() -> arm.setRotation(StagingState.CORAL_L4.rotation), arm)
        );
    }

    public static SequentialCommandGroup L4_Falling(Elevator elevator, Arm arm, Intake intake){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> elevator.setExtension(4.0307), elevator),
            Commands.runOnce(() -> intake.setCommandOutake(true), intake),
            Commands.runOnce(() -> intake.L4Outake(), intake),
            Commands.runOnce(() -> arm.setRawVoltageOut(-0.2), arm),
            new WaitUntilCommand(() -> (arm.getRotation() > 0.28418)),
            Commands.runOnce(() -> intake.stop(), intake),
            Commands.runOnce(() -> intake.setCommandOutake(false), intake),
            all(StagingState.ZEROED, elevator, arm)
        );
    }

    public static ParallelCommandGroup all(StagingState state, Elevator elevator, Arm arm){
        return new ParallelCommandGroup(
            new RotateTo(state.rotation, () -> elevator.isInwardsRotationSafe(), arm),
            new ExtendTo(state.extension, () -> arm.isExtensionSafe(), elevator)
        );
    }

    public static SequentialCommandGroup zero(Elevator elevator, Arm arm, Intake intake){
        ParallelCommandGroup hasAlgae = !intake.hasAlgae()
            ? new ParallelCommandGroup(
                new RotateTo(StagingState.ZEROED.rotation, () -> elevator.isInwardsRotationSafe(), arm),
                new ExtendTo(StagingState.ZEROED.extension, () -> arm.isExtensionSafe(), elevator))
            : new ParallelCommandGroup(
                new RotateTo(StagingState.ALGAE_SAFE.rotation, () -> elevator.isInwardsRotationSafe(), arm),
                new ExtendTo(StagingState.ALGAE_SAFE.extension, () -> arm.isExtensionSafe(), elevator));
        
        return new SequentialCommandGroup(
            hasAlgae,
            Commands.runOnce(() -> elevator.setCoast(), elevator)
        );
    }

    public static SequentialCommandGroup shotput(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.sequence(
                new RotateTo(StagingState.CORAL_L1.rotation, () -> elevator.isInwardsRotationSafe(), arm),
                new ExtendTo(StagingState.CORAL_L1.extension, () -> arm.isExtensionSafe(), elevator)),
            Commands.runOnce(() -> elevator.setCoast(), elevator)
        );
    }
}
