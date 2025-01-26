package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class SuperStructureStaging {

    public class L4_Coral extends SequentialCommandGroup {
        
        public final Elevator elevator;
        public final Arm arm;

        public L4_Coral(Elevator elevator, Arm arm){
            this. elevator = elevator;
            this.arm = arm;

            addRequirements(elevator, arm);
        }       
    }

    public static void REST(){
        
    }
}
