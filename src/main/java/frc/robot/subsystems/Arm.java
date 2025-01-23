package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    
    private final TalonFX pivotMotor = new TalonFX(0); //TODO: ID ME
    private final TalonFX intaekeMotor = new TalonFX(0);

    // sensors :(

    public Arm(){

    }

    public void rawIntake(boolean coralIntake){
        // spins motor
    }

    public void rotateToDegrees(double desiredPosition){

    }

    @Override
    public void periodic() {
        // runs every 20 ms
    }

}
