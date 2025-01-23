package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    
    private final TalonFX extensionLeader = new TalonFX(0); // TODO: ID ME :(
	private final TalonFX extensionFollower = new TalonFX(0);

    private CANcoder kExtensionEncoder = new CANcoder(0);

    public Elevator(){
        
    }

    public void rawExtension(boolean extending){

    }

    @Override
    public void periodic() {
        
    }
}
