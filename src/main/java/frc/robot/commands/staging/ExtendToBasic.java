package frc.robot.commands.staging;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.StagingManager;
import frc.robot.subsystems.Elevator;

public class ExtendToBasic extends Command {
    private final Elevator elevator;
    private final BooleanSupplier extensionSafe;
    private final double targetExtension;
    private final boolean extendsPast; 

    public ExtendToBasic(double extension, BooleanSupplier inwardsRotationSafe, Elevator elevator) {
        this.targetExtension = extension;
        this.extensionSafe = inwardsRotationSafe;
        this.elevator = elevator;
        this.extendsPast = extension > Constants.Elevator.kInwardsRotationSafeExtension;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setExtension( 
            extendsPast && !extensionSafe.getAsBoolean() 
            ? StagingManager.StagingState.ZEROED.extension
            : targetExtension); 
    }
    
    @Override
    public boolean isFinished() {
        return MathUtil.isNear(targetExtension, elevator.getExtension(), Constants.Elevator.kValidExtensionTolerance);
    }
}