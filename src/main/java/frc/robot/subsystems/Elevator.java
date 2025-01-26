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
import com.redstorm509.stormkit.math.PositionTarget;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    
    private final TalonFX extensionLeader = new TalonFX(Constants.IDs.kExtensionLeader); // TODO: ID ME :(
	private final TalonFX extensionFollower = new TalonFX(Constants.IDs.kExtensionFollower);

    private CANcoder extensionEncoder = new CANcoder(0);
    
    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(0, 0));
    private final Timer timer = new Timer(); 

    private VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);
	private PositionVoltage closedLoopPosition = new PositionVoltage(0).withEnableFOC(false);

    private enum ExtensionState {
        // Defaultst                                    
        ZEROED(new State()),

        // Coral
        CORAL_L4(new State()),
        CORAL_L3(new State()),
        CORAL_L2(new State()),
        CORAL_L1(new State()),
        CORAL_GROUND(new State()),

        // Algae
        ALGAE_HIGH(new State()),
        ALGAE_LOW(new State()),
        ALGAE_GROUND(new State());


        public final State state;

        ExtensionState(State state){
            this.state = state;
        }
    }

    public ExtensionState currentExtensionState;
    public ExtensionState targetExtensionState;


    public Elevator(){
        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();

        // Current limits
		extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		extensionConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kElevatorSupply; 
        extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kElevatorStator;

        // PID value assignment
		extensionConfig.Slot0.kP = Constants.PIDConstants.Elevator.kExtensionP;
		extensionConfig.Slot0.kI = Constants.PIDConstants.Elevator.kExtensionI;
		extensionConfig.Slot0.kD = Constants.PIDConstants.Elevator.kExtensionD;
		extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Assigning encoder as feedback device
		extensionConfig.Feedback.FeedbackRemoteSensorID = extensionEncoder.getDeviceID();
		extensionConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		extensionConfig.Feedback.RotorToSensorRatio = Constants.Elevator.kExtensionGearRatio;

		extensionLeader.getConfigurator().apply(extensionConfig);
		extensionFollower.getConfigurator().apply(extensionConfig);
		extensionFollower.setControl(new Follower(extensionLeader.getDeviceID(), true)); // TODO: Check design

		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		encoderConfig.MagnetSensor.MagnetOffset = (Constants.Elevator.kExtensionMagnetOffset) / 360.0;
		encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
		extensionEncoder.getConfigurator().apply(encoderConfig);
    }

    public void rawExtension(boolean extending){
        extensionLeader.setControl(openLoop.withOutput(null));
    }

    public void setTargetState(ExtensionState target){
        targetExtensionState = target;
    }

    @Override
    public void periodic() {
        
    }
}
