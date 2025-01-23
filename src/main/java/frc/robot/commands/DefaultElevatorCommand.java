package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class DefaultElevatorCommand extends Command {

    private final Elevator elevator;
    private final DoubleSupplier extensionSupplier;

    public DefaultElevatorCommand(Elevator elevator, DoubleSupplier extensionSupplier){
        this.elevator = elevator;
        this.extensionSupplier = extensionSupplier;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        //
    }
}
