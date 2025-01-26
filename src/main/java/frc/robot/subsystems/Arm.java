package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    
    private final TalonFX pivotMotor = new TalonFX(Constants.IDs.kPivotMotor);
    private final TalonFX intakeMotor = new TalonFX(Constants.IDs.kIntakeMotor);

    // sensors :(
    private final CANcoder pivotEncoder = new CANcoder(Constants.IDs.kPivotEncoder);
    
    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(0, 0));
    private State tpGoal = new State(0,0);
    private State tpCurrent = new State(0,0);
    private final Timer timer = new Timer();

    private final VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);
	private final PositionVoltage closedLoopPosition = new PositionVoltage(0).withEnableFOC(false);

    private boolean intaking;

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
		pivotConfig.Feedback.RotorToSensorRatio = Constants.Arm.kRotationGearRatio;

		pivotMotor.getConfigurator().apply(pivotConfig);
		
        // rotation encoder
		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		encoderConfig.MagnetSensor.MagnetOffset = (Constants.Arm.kPivotMagnetOffset) / 360.0;
		encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // change
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
		pivotEncoder.getConfigurator().apply(encoderConfig);

        // Make starting position zero


        // Intake 
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        // Current limits
		pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kIntakeSupply; 
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kIntakeStator;

        // PID value assignment
		pivotConfig.Slot0.kP = Constants.PIDConstants.Arm.kIntakeP;
		pivotConfig.Slot0.kI = Constants.PIDConstants.Arm.kIntakeI;
		pivotConfig.Slot0.kD = Constants.PIDConstants.Arm.kIntakeD;
		pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		intakeMotor.getConfigurator().apply(intakeConfig);
    }

    public void rawIntake(boolean intaking, boolean coralIntake){
        if (coralIntake){ // TODO: find which is inverted
            intakeMotor.setControl(openLoop.withOutput(Constants.Arm.kIntakeSpeed * 12.0));
        } else {
            intakeMotor.setControl(openLoop.withOutput(-Constants.Arm.kIntakeSpeed * 12.0));
        }
    }
    
    /**
     * @param desiredRotation The target angle, in degrees from the starting configuration (0 degrees)
     */
    public void rotateToDegrees(double desiredRotation){
        desiredRotation %= 360; // Ensures a degree value within 360 degrees, within reason
        desiredRotation /= 360.0; // Changes degree rotation to 
        pivotMotor.setControl(closedLoopPosition.withPosition(desiredRotation));
    }

    public void setTargetRotation(double desiredRotation){
        tpGoal = new State(desiredRotation,0);
    }

    /**
     * @return The rotation of the arm from the starting configuration, in degrees 
     */
    public double getRotation(){
        return pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    public boolean isExtensionSafe(){
        return getRotation() > 0.0; // TODO: MUST FIX AND ADD REAL VALUE RAHHHHH 
    }

    @Override
    public void periodic() {
        tpCurrent = trapezoidProfile.calculate(0.02, tpCurrent, tpGoal);
        
        // Send setpoint to offboard controller PID
        pivotMotor.setControl(closedLoopPosition.withPosition(null).withEnableFOC(true));
        rotateToDegrees(tpGoal.position);

        if (Math.abs(intakeMotor.getVelocity().getValueAsDouble()) <= 0.8 * Constants.Arm.kIntakeSpeed * 512.0 && intaking){
            
        }
    }
}
