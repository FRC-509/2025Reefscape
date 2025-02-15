package frc.robot.commands.staging;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class RotateTo extends Command {
    public final Arm arm;
    public final BooleanSupplier inwardsRotationSafe;
    public final double targetRotation;

    public TrapezoidProfile trapezoidProfile;
    private State tpGoal;
    private State tpCurrent;
    private boolean resetMotionProfile;

    public RotateTo(double rotation, BooleanSupplier inwardsRotationSafe, Arm arm) {
        this.targetRotation = rotation;
        this.inwardsRotationSafe = inwardsRotationSafe;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        this.trapezoidProfile = new TrapezoidProfile(Constants.Arm.kMotionProfileConstraints);
        tpGoal = new State(targetRotation, 0.0);
        tpCurrent = new State(arm.getRotation(), arm.getRotatingVelocity());
        resetMotionProfile = false;
    }

    @Override
    public void execute() {
        if (inwardsRotationSafe.getAsBoolean()) {
            if (resetMotionProfile) {
                trapezoidProfile = new TrapezoidProfile(Constants.Elevator.kMotionProfileConstraints);
                tpCurrent = new State(arm.getRotation(), arm.getRotatingVelocity());
                resetMotionProfile = false;
            }
            tpCurrent = trapezoidProfile.calculate(0.02, tpCurrent, tpGoal);
            // arm.setRotationVelocity(targetRotation); 
        } else {
            resetMotionProfile = true;
            // arm.setRotationVelocity(0.0);
        }
    }
    
    @Override
    public boolean isFinished() {
        return MathUtil.isNear(targetRotation, arm.getRotation(), Constants.Arm.kValidRotationTolerance);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) arm.maintainRotation();
    }
}