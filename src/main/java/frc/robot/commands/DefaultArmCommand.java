package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class DefaultArmCommand extends Command {
    
    private final Arm arm; 
    private final DoubleSupplier rotationSupplier;

    public DefaultArmCommand(Arm arm, DoubleSupplier rotationSupplier){
        this.arm = arm;
        this.rotationSupplier = rotationSupplier;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        
    }
}
