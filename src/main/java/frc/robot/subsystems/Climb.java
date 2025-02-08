package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.introspect.AnnotationCollector.OneAnnotation;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    
    private final TalonFX climbMotor = new TalonFX(Constants.IDs.kClimbMotor);
    private final PositionVoltage rotateClosedLoop = new PositionVoltage(climbMotor.getPosition().getValueAsDouble()).withEnableFOC(false);

	private Solenoid lockSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.IDs.kClimbSolenoid);
    private boolean lockClimb;

    public Climb() {
        lockClimb = false;
        TalonFXConfiguration climbConfig = new TalonFXConfiguration();

        // Current limits
		climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		climbConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kClimbSupply; 
        climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        climbConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kClimbStator;

        // PID value assignment
		climbConfig.Slot0.kP = Constants.PIDConstants.Climb.kRotateP;
		climbConfig.Slot0.kI = Constants.PIDConstants.Climb.kRotateI;
		climbConfig.Slot0.kD = Constants.PIDConstants.Climb.kRotateD;
		climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Feedback
        climbConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

		climbMotor.getConfigurator().apply(climbConfig);
    }

    /**
     * Sets the position target of the climber to it's vertical state  
     */
    public void setClimbPivot(){
        climbMotor.setControl(rotateClosedLoop.withPosition(0));
    }

    /**
     * Sets the position of the climber 
     */
    public void setPassivePivot(){
        climbMotor.setControl(rotateClosedLoop.withPosition(0));
    }

    public void toggleLockClimb(){
        lockClimb = !lockClimb;
        lockSolenoid.set(lockClimb);
    }

    public void overideLock(boolean lock){
        lockSolenoid.set(lock);
    }
}
