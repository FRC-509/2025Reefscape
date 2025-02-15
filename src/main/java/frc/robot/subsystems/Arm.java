package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.StagingManager.StagingState;

public class Arm extends SubsystemBase {
    
    private final TalonFX pivotMotor = new TalonFX(Constants.IDs.kPivotMotor, Constants.kCANIvore);

    // sensors :(
    private final CANcoder pivotEncoder = new CANcoder(Constants.IDs.kPivotEncoder, Constants.kCANIvore);

	// private final PositionVoltage closedLoopPosition = new PositionVoltage(0).withEnableFOC(false);
    // private final VelocityVoltage closedLoopVelocity = new VelocityVoltage(0.0).withEnableFOC(false);
    private final PositionDutyCycle ddd = new PositionDutyCycle(pivotMotor.getRotorPosition().getValueAsDouble());

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
		
        // rotation encoder
		// CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		// encoderConfig.MagnetSensor.MagnetOffset = (Constants.Arm.kPivotMagnetOffset) / 360.0;
		// encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // change
        // encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    
		// pivotEncoder.getConfigurator().apply(encoderConfig);

        // Make starting position zero
    }

    /**
     * @return The rotation of the arm from the starting configuration, in degrees 
     */
    public double getRotation(){
        return pivotMotor.getPosition().getValueAsDouble();
    }

    /**
     * @return The rotations per second of the arm
     */
    public double getRotatingVelocity() {
        return pivotMotor.getVelocity().getValueAsDouble();
    }

    public void setRotation(double state){
        pivotMotor.setControl(ddd.withPosition(state));
    }

    /**
     * @param velocity Desired rotational velocity in rotations per second
    //  */
    // public void setRotationVelocity(double velocity) {
    //     double rotationsPerSecond = velocity;
    //     pivotMotor.setControl(closedLoopVelocity.withVelocity(rotationsPerSecond));
    // }

    public void maintainRotation() {
        // pivotMotor.setControl(closedLoopPosition.withPosition(pivotMotor.getPosition().getValueAsDouble())); 
    }

    public boolean isExtensionSafe(){
        // Rotation is negative
        return getRotation() < Constants.Arm.kExtensionSafeAngle; // TODO: MUST FIX AND ADD REAL VALUE RAHHHHH
    }

    @Override
    public void periodic() {
        dashboard();
    }

    private void dashboard(){
        // SmartDashboard.putNumber("Arm Absolute positiion", pivotEncoder.getAbsolutePosition().getValueAsDouble());
        // SmartDashboard.putNumber("Arm positiion", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm rotor positiion", pivotMotor.getRotorPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Arm Extension Safe", isExtensionSafe());
    }
}
