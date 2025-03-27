package frc.robot.commands.staging;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class RotateTo extends Command {
    private final Arm arm;
    private final BooleanSupplier inwardsRotationSafe;
    private final double targetRotation;
    private final boolean rotatesInwards;

    public RotateTo(double rotation, BooleanSupplier inwardsRotationSafe, Arm arm) {
        this.targetRotation = rotation;
        this.inwardsRotationSafe = inwardsRotationSafe;
        this.arm = arm;
        this.rotatesInwards = rotation < StagingManager.kExtensionSafeRotation;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // if the arm wants to rotate inwards, but it is currently unsafe to do so, it will
        // rotate to the closest safe position
        if (rotatesInwards && !inwardsRotationSafe.getAsBoolean()){
	        arm.setRotation(StagingManager.StagingState.SAFE.rotation);
        // } else if(targetRotation > StagingManager.kGroundRotation){
        //     arm.setRotation(StagingManager.StagingState.SAFE.rotation);
        } else { // otherwise, it is safe to rotate to any orientation
	        arm.setRotation(targetRotation);
        }
    }
    
    @Override
    public boolean isFinished() {
        return MathUtil.isNear(targetRotation, arm.getRotation(), Constants.Arm.kValidRotationTolerance);
    }
}