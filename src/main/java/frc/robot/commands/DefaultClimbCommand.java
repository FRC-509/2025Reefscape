package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class DefaultClimbCommand extends Command {
    
    private final Climber climber;
    private final DoubleSupplier pivotSupplier;
    private final BooleanSupplier autoPivotSupplier;

    private boolean autoPivoting;
    private boolean alreadySetPivot;

    public DefaultClimbCommand(DoubleSupplier pivotSupplier, BooleanSupplier autoPivotSupplier, Climber climber) {
        this.climber = climber;
        this.pivotSupplier = pivotSupplier;
        this.autoPivotSupplier = autoPivotSupplier;
        this.autoPivoting = false;
        this.alreadySetPivot = false;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (autoPivotSupplier.getAsBoolean()) {
            autoPivoting = true;
            alreadySetPivot = false;
        }
        
        if (Math.abs(pivotSupplier.getAsDouble()) > Constants.Operator.kStickDeadband){
            // climber.pivot(pivotSupplier.getAsDouble());
            autoPivoting = false;
        } else {
            if (autoPivoting && !alreadySetPivot
                && !MathUtil.isNear(Constants.Climber.kClimbPositionDegrees, climber.getPivotDegrees(), Constants.Climber.kValidRotationTolerance)) {
                alreadySetPivot = true;
                climber.setClimbPivot();
            }
        }
    }

}
