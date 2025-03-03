package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    
    private final TalonFX pivotMotor = new TalonFX(Constants.IDs.kPivotMotor, Constants.kCANIvore);
    private final CANcoder pivotEncoder = new CANcoder(Constants.IDs.kPivotEncoder, Constants.kCANIvore);
    private final PositionDutyCycle closedLoopPosition = new PositionDutyCycle(pivotMotor.getPosition().getValueAsDouble());

    public Arm(){
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        // Current limits
		pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kArmSupply; 
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kArmStator;

        // PID value assignment
		pivotConfig.Slot0.kP = Constants.PIDConstants.Arm.kRotationP;
		pivotConfig.Slot0.kI = Constants.PIDConstants.Arm.kRotationI;
		pivotConfig.Slot0.kD = Constants.PIDConstants.Arm.kRotationD;
		pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Assigning encoder as feedback device
		pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
		pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		pivotConfig.Feedback.RotorToSensorRatio = Constants.Arm.kRotorToSensorRatio;
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.Arm.kSensorToMechanismRatio;

		pivotMotor.getConfigurator().apply(pivotConfig);
    }

    public double getRotation(){
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public void setRotation(double position){
        pivotMotor.setControl(closedLoopPosition.withPosition(position));
    }

    public boolean isExtensionSafe(){
        // Outwards rotation is negative
        return getRotation() < Constants.Arm.kExtensionSafeAngle;
    }

    @Override
    public void periodic() {
        dashboard();
    }

    private void dashboard(){
        SmartDashboard.putNumber("Arm rotor positiion", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Arm Extension Safe", isExtensionSafe());
    }
}
