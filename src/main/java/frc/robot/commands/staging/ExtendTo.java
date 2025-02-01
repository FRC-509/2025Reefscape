package frc.robot.commands.staging;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ExtendTo extends Command {
    public final Elevator elevator;
    public final BooleanSupplier extensionSafe;
    public final double targetExtension;

    public TrapezoidProfile trapezoidProfile;
    private State tpGoal;
    private State tpCurrent;
    private boolean resetMotionProfile;

    public ExtendTo(double extension, BooleanSupplier extensionSafe, Elevator elevator) {
        this.targetExtension = extension;
        this.extensionSafe = extensionSafe;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        tpGoal = new State(targetExtension, 0.0);
        tpCurrent = new State(elevator.getExtension(), elevator.getExtendingVelocity());
        resetMotionProfile = false;
    }

    @Override
    public void execute() {
        if (extensionSafe.getAsBoolean()) {
            if (resetMotionProfile) {
                trapezoidProfile = new TrapezoidProfile(Constants.Elevator.kMotionProfileConstraints);
                tpCurrent = new State(elevator.getExtension(), elevator.getExtendingVelocity());
                resetMotionProfile = false;
            }
            tpCurrent = trapezoidProfile.calculate(0.02, tpCurrent, tpGoal);
            elevator.setExtendingVelocity(tpCurrent.velocity); 
        } else {
            resetMotionProfile = true;
            elevator.setExtendingVelocity(0.0);
        }
    }
    
    @Override
    public boolean isFinished() {
        return MathUtil.isNear(targetExtension, elevator.getExtension(), Constants.Elevator.kValidStateTolerance);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) elevator.maintainExntension();
    }
}