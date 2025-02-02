package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    private final TalonFX intakeMotor = new TalonFX(Constants.IDs.kIntakeMotor);
    private final VoltageOut intakeOpenLoop = new VoltageOut(0).withEnableFOC(false);

    public Intake() {
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        // Current limits
		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kIntakeSupply; 
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kIntakeStator;

        // PID value assignment
		intakeConfig.Slot0.kP = Constants.PIDConstants.Arm.kIntakeP;
		intakeConfig.Slot0.kI = Constants.PIDConstants.Arm.kIntakeI;
		intakeConfig.Slot0.kD = Constants.PIDConstants.Arm.kIntakeD;
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		intakeMotor.getConfigurator().apply(intakeConfig);
    }


    public void intake(boolean algaeInwards){
        if (algaeInwards){ // TODO: find which is inverted
            intakeMotor.setControl(intakeOpenLoop.withOutput(Constants.Arm.kIntakeSpeed * 12.0));
        } else {
            intakeMotor.setControl(intakeOpenLoop.withOutput(-Constants.Arm.kIntakeSpeed * 12.0));
        }
    }

    public void stopIntaking() {
        intakeMotor.setControl(intakeOpenLoop.withOutput(Constants.Arm.kIntakeSpeed * 12.0));
    }

    // @Override
    // public void periodic() {
    //     if (Math.abs(intakeMotor.getVelocity().getValueAsDouble()) <= 0.8 * Constants.Arm.kIntakeSpeed * 512.0){
    //         stopIntaking();
    //     }
    // }
}
