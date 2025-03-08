package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.StagingManager;

public class Climber extends SubsystemBase {
    
    private final TalonFX climbMotor = new TalonFX(Constants.IDs.kClimbMotor);
    private final PositionVoltage rotationClosedLoop = new PositionVoltage(climbMotor.getPosition().getValueAsDouble()).withEnableFOC(false);
    private final VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);

	private Solenoid lockSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.IDs.kClimbSolenoid);
    private boolean lockClimb;
    private double initialRotation;

    public Climber() {
        lockClimb = false;
        TalonFXConfiguration climbConfig = new TalonFXConfiguration();

        // Current limits
		climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		climbConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kClimbSupply; 
        climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        climbConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kClimbStator;

        // PID value assignment
		climbConfig.Slot0.kP = Constants.PIDConstants.Climber.kRotateP;
		climbConfig.Slot0.kI = Constants.PIDConstants.Climber.kRotateI;
		climbConfig.Slot0.kD = Constants.PIDConstants.Climber.kRotateD;
		climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Feedback
        climbConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        climbConfig.Feedback.SensorToMechanismRatio = Constants.Climber.kSensorToMechanismRatio;

		climbMotor.getConfigurator().apply(climbConfig);
    }

    /**
     * Sets the position target of the climber to it's most vertical state  
     */
    public void setClimbPivot(){
        climbMotor.setControl(rotationClosedLoop.withPosition(Constants.Climber.kClimbPositionDegrees));
    }

    public double getPivotDegrees() {
		return climbMotor.getPosition().getValueAsDouble() * 360.0;
	}

    public double getRotation(){
        return initialRotation - climbMotor.getPosition().getValueAsDouble();
    }

    public void setRotation(double position){
        position = initialRotation - position;
        climbMotor.setControl(rotationClosedLoop.withPosition(position));
    }

    public boolean isExtensionSafe(){
        return getRotation() > StagingManager.kExtensionSafeRotation;
    }

    public void setCoast(){

    }

    public void setInitializationRotation(){
        this.initialRotation = climbMotor.getPosition().getValueAsDouble();
    }

    // public boolean isZeroed() {
    //     return limitSwitch.get();
    // }

    public void toggleLockClimb(){
        lockClimb = !lockClimb;
        lockSolenoid.set(lockClimb);
    }

    public void overrideLock(boolean lock){
        lockSolenoid.set(lock);
        lockClimb = lock;
    }

}
