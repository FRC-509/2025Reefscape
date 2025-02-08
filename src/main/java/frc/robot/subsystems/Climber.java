package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    
    private final TalonFX climbMotor = new TalonFX(Constants.IDs.kClimbMotor);
    private final PositionVoltage rotationClosedLoop = new PositionVoltage(climbMotor.getPosition().getValueAsDouble()).withEnableFOC(false);

	private Solenoid lockSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.IDs.kClimbSolenoid);
    private boolean lockClimb;

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

    /**
     * @param rotationDeriv The desired rate of change of the climber's postion,
     *                      represented as a value from 0.0 to 1.0, with 1.0 being 
     *                      the maximum rotational speed of the climber 
     */
    public void pivot(double rotationDeriv){
        setPivotDegrees(getPivotDegrees() + Constants.Climber.kMaxRotationalSpeed * rotationDeriv);
    }

    public double getPivotDegrees() {
		return climbMotor.getPosition().getValueAsDouble() * 360.0;
	}

	public void setPivotDegrees(double targetDegrees) {
		double delta = (targetDegrees - getPivotDegrees()) % 360;

		if (delta > 180.0d) {
			delta -= 360.0d;
		} else if (delta < -180.0d) {
			delta += 360.0d;
		}

		double target = getPivotDegrees() + delta;
		double ticks = target / 360.0d;

		climbMotor.setControl(rotationClosedLoop.withPosition(ticks));
	}

    public void toggleLockClimb(){
        lockClimb = !lockClimb;
        lockSolenoid.set(lockClimb);
    }

    public void overideLock(boolean lock){
        lockSolenoid.set(lock);
        lockClimb = lock;
    }
}
