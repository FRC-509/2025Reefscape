package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakingState;
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
        BooleanSupplier alternateCondition;
        Runnable algaeAlternate;

        public StagingTrigger(
            BooleanSupplier booleanSupplier, 
            Runnable onTrue,
            Runnable onFalse){
            this.booleanSupplier = booleanSupplier;
            this.last = false;
            this.onTrue = onTrue;
            this.algaeAlternate = onTrue;
            this.onFalse = onFalse;
            this.alternateCondition = () -> false;
        }

        public StagingTrigger(
            BooleanSupplier booleanSupplier, 
            Runnable onTrue,
            Runnable onFalse,
            Runnable algaeAlternate,   
            BooleanSupplier alternateCondition){
            this.booleanSupplier = booleanSupplier;
            this.last = false;
            this.onTrue = onTrue;
            this.algaeAlternate = algaeAlternate;
            this.onFalse = onFalse;
            this.alternateCondition = alternateCondition;
        }
    }

    // Stage Triggers
    private final StagingTrigger L4_StagingTrigger;
    private final StagingTrigger L3_StagingTrigger;
    private final StagingTrigger L2_StagingTrigger;
    private final StagingTrigger L1_StagingTrigger;

    private final StagingTrigger coralStation_StagingTrigger;
    
    private final StagingTrigger coralGround_StagingTrigger;
    private final StagingTrigger algaeGround_StagingTrigger; 

    private final DoubleSupplier extensionSupplier;
    private final DoubleSupplier extensionDistance;

    private Runnable quedStage;
    private final Runnable safeZero;
    private final Runnable relaxElevator;
    private boolean buttonIsPressed;

    // Manual override
    private final StagingTrigger manualZero_StagingTrigger;
    private final StagingTrigger manualSafeZero_StagingTrigger;
    private final BooleanSupplier softResetSupplier;
    private final BooleanSupplier hardResetSupplier;
    private final Runnable softReset;
    private final Runnable hardReset;
    private boolean reseting;
    private boolean coralL4;

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
        BooleanSupplier manualZeroSupplier,
        BooleanSupplier manualSafeZeroSupplier,
        BooleanSupplier softResetSupplier,
        BooleanSupplier hardResetSupplier
    ){
        this.L4_StagingTrigger = new StagingTrigger(
            L4_Supplier,
            () -> {
                coralL4 = !(intake.getIntakingState().equals(IntakingState.ALGAE_PASSIVE)
                    || intake.getIntakingState().equals(IntakingState.ALGAE_INTAKE) 
                    || intake.getIntakingState().equals(IntakingState.ALGAE_OUTAKE));
                L4_Rising(elevator, arm).schedule();
            },
            () -> L4_Falling(elevator, arm, intake, () -> coralL4).schedule());

        this.L3_StagingTrigger = new StagingTrigger(
            L3_Supplier,
            () -> allSafe(StagingState.CORAL_L3, elevator, arm).schedule(),
            () -> zero(elevator, arm, intake).schedule(),
            () -> allSafe(StagingState.ALGAE_HIGH, elevator, arm).schedule(),
            () -> {
                return intake.getIntakingState().equals(IntakingState.ALGAE_INTAKE)
                    || intake.getIntakingState().equals(IntakingState.ALGAE_PASSIVE);
            });

        this.L2_StagingTrigger = new StagingTrigger(
            L2_Supplier,
            () -> allSafe(StagingState.CORAL_L2, elevator, arm).schedule(),
            () -> zero(elevator, arm, intake).schedule(),
            () -> allSafe(StagingState.ALGAE_LOW, elevator, arm).schedule(),
            () -> {
                return intake.getIntakingState().equals(IntakingState.ALGAE_INTAKE)
                    || intake.getIntakingState().equals(IntakingState.ALGAE_PASSIVE);
            });

        this.L1_StagingTrigger = new StagingTrigger(
            L1_Supplier,
            () -> allSafe(StagingState.CORAL_L1, elevator, arm).schedule(), 
            () -> zero(elevator, arm, intake).schedule());

        this.coralStation_StagingTrigger = new StagingTrigger(
            coralStation_Supplier,
            () -> all(StagingState.CORAL_STATION, elevator, arm).schedule(), 
            () -> zero(elevator, arm, intake).schedule());

        this.coralGround_StagingTrigger = new StagingTrigger(
            coralGround_Supplier, 
            () -> all(StagingState.CORAL_GROUND, elevator, arm).schedule(),
            () -> zero(elevator, arm, intake).schedule(),
            () -> all(StagingState.LOLIPOP, elevator, arm).schedule(),
            () -> {
                return intake.getIntakingState().equals(IntakingState.ALGAE_INTAKE)
                    || intake.getIntakingState().equals(IntakingState.ALGAE_PASSIVE);
            });

        this.algaeGround_StagingTrigger = new StagingTrigger(
            algaeGround_Supplier,
            () -> all(StagingState.ALGAE_GROUND, elevator, arm).schedule(),
            () -> zero(elevator, arm, intake).schedule(),
            () -> all(StagingState.PROCESSOR, elevator, arm).schedule(),
            () -> {
                return intake.getIntakingState().equals(IntakingState.ALGAE_PASSIVE)
                    || intake.getIntakingState().equals(IntakingState.ALGAE_OUTAKE);
            });
        
        this.extensionSupplier = () -> elevator.getExtension();
        this.extensionDistance = () -> elevator.getDistanceOffGround();
        this.quedStage = null;
        this.buttonIsPressed = false;
        this.safeZero = new Runnable() {
            @Override
            public void run() {
                (new SequentialCommandGroup(
                    Commands.runOnce(() -> arm.setRotation(StagingManager.StagingState.SAFE.rotation), arm),
                    zero(elevator, arm, intake)
                )).schedule();
            }
        };
        this.relaxElevator = new Runnable() {
            @Override
            public void run() {
                elevator.setCoast();
            }
        };
        
        this.manualZero_StagingTrigger = new StagingTrigger(
            manualZeroSupplier, 
            () -> zero(elevator, arm, intake).schedule(),
            () -> { reseting = false; });           
        this.manualSafeZero_StagingTrigger = new StagingTrigger(
                manualZeroSupplier, 
                () -> safeZero.run(),
                () -> { reseting = false; });     
        this.softReset = new Runnable() {
            @Override
            public void run() {
                softResetSuperstructure(elevator, arm).schedule();
            }
        };        
        this.hardReset = new Runnable() {
            @Override
            public void run() {
                hardstopSuperstructure(elevator, arm).schedule();
            }
        };
        this.softResetSupplier = softResetSupplier;
        this.hardResetSupplier = hardResetSupplier;
    }

    public void update(){
        onChange(manualZero_StagingTrigger);
        onChange(manualSafeZero_StagingTrigger);
        if (softResetSupplier.getAsBoolean()) {
            softReset.run();
            reseting = true;
            return;
        }
        if (hardResetSupplier.getAsBoolean()) {
            hardReset.run();
            reseting = true;
            return;
        }
        if (reseting) return;

        onChange(L3_StagingTrigger);
        onChange(L2_StagingTrigger);
        onChange(L1_StagingTrigger);

        onChange(coralStation_StagingTrigger);
        onChange(coralGround_StagingTrigger);
        onChange(algaeGround_StagingTrigger);

        onChange(L4_StagingTrigger);

        if (L4_StagingTrigger.booleanSupplier.getAsBoolean()
            || L3_StagingTrigger.booleanSupplier.getAsBoolean()
            || L2_StagingTrigger.booleanSupplier.getAsBoolean()
            || L1_StagingTrigger.booleanSupplier.getAsBoolean()
            || coralStation_StagingTrigger.booleanSupplier.getAsBoolean()
            || coralGround_StagingTrigger.booleanSupplier.getAsBoolean()
            || algaeGround_StagingTrigger.booleanSupplier.getAsBoolean()){
                buttonIsPressed = true;
            } else buttonIsPressed = false;
        if (quedStage != null && extensionSupplier.getAsDouble() < StagingManager.StagingState.ALGAE_GROUND.extension){
            quedStage.run();
            quedStage = null;
        } else if (quedStage != null && extensionDistance.getAsDouble() < 0.07 && !buttonIsPressed) relaxElevator.run();
    }

    void onChange(StagingTrigger trigger) {
        if (!trigger.last && trigger.booleanSupplier.getAsBoolean()){ // on true
            quedStage = !trigger.alternateCondition.getAsBoolean() ?  trigger.onTrue : trigger.algaeAlternate;
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
        SAFE(0.111523,0.257803),
        ALGAE_SAFE(0.111523, 0.2233887),

        // Coral
        CORAL_L4(4.777822,0.1240143),
        CORAL_L3(4.758,0.485),
        CORAL_L2(3.367207,0.485),
        CORAL_L1(1.8645,0.426),

        // Algae
        ALGAE_HIGH(4.0967207,0.48338),
        ALGAE_LOW(2.5645,0.48338),

        // Ground Pickup
        ALGAE_GROUND(0.3322,0.474609),
        CORAL_GROUND(0.3828,0.448402),
        LOLIPOP(1.43,0.474609),

        // Field Elements
        CORAL_STATION(1.938,0.304195),
        PROCESSOR(1.022,0.474609);


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

    public static SequentialCommandGroup L4_Falling(Elevator elevator, Arm arm, Intake intake, BooleanSupplier coralSupplier){
        return coralSupplier.getAsBoolean()
            ? new SequentialCommandGroup(
                Commands.runOnce(() -> elevator.setExtension(4.0307), elevator),
                Commands.runOnce(() -> arm.setRawVoltageOut(-0.2), arm),
                Commands.runOnce(() -> intake.L4Outake()),
                new WaitUntilCommand(() -> (arm.getRotation() > 0.21418)),
                zero(elevator, arm, intake))
            : new SequentialCommandGroup(
                Commands.parallel(
                    Commands.runOnce(() -> elevator.setExtension(4.0307), elevator),
                    Commands.runOnce(() -> arm.setRotation(0.28418), arm)),
                zero(elevator, arm, intake));
    }

    public static ParallelCommandGroup all(StagingState state, Elevator elevator, Arm arm){
        return new ParallelCommandGroup(
            new RotateTo(state.rotation, () -> elevator.isInwardsRotationSafe(), arm),
            new ExtendTo(state.extension, () -> arm.isExtensionSafe(), elevator)
        );
    }

    public static SequentialCommandGroup allSafe(StagingState state, Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> arm.setRotation(StagingState.SAFE.rotation), arm),
            Commands.waitSeconds(0.4),
            all(state, elevator, arm)
        );
    }

    public static SequentialCommandGroup algaeAlternate(boolean isHigh, Elevator elevator, Arm arm, Intake intake){
        StagingState state = isHigh ? StagingState.ALGAE_HIGH : StagingState.ALGAE_LOW;
        return new SequentialCommandGroup(
            allSafe(state, elevator, arm),
            intake.getIntakingState().equals(IntakingState.ALGAE_INTAKE) 
                ? Commands.runOnce(() -> elevator.setExtension(isHigh 
                        ? StagingState.ALGAE_HIGH.extension 
                        : StagingState.ALGAE_LOW.extension), 
                    elevator)
                : Commands.none()
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

    public static SequentialCommandGroup softResetSuperstructure(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> arm.setRawVoltageOut(0.725), arm),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> arm.setCoast(), arm),
            hardstopSuperstructure(elevator, arm)
        );
    }

    public static SequentialCommandGroup hardstopSuperstructure(Elevator elevator, Arm arm){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> arm.setCoast(), arm),
            Commands.runOnce(() -> elevator.setRawVoltageOut(-1.25), elevator),
            Commands.waitUntil(() -> elevator.isZeroed()),
            Commands.runOnce(() -> elevator.setCoast(), elevator),
            Commands.runOnce(() -> arm.setRawVoltageOut(0.725), arm),
            Commands.waitUntil(() -> arm.isZeroed()),
            Commands.runOnce(() -> arm.setCoast(), arm)
        );
    }
}
