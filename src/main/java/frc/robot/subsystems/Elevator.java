package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.StagingManager;

public class Elevator extends SubsystemBase {
    
    private final TalonFX extensionLeader = new TalonFX(Constants.IDs.kExtensionLeader); // TODO: ID ME :(
	private final TalonFX extensionFollower = new TalonFX(Constants.IDs.kExtensionFollower);

    private CANcoder extensionEncoder = new CANcoder(0);

    private VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);
	private PositionVoltage closedLoopPosition = new PositionVoltage(0).withEnableFOC(false);
    private VelocityVoltage closedLoopVelocity = new VelocityVoltage(0).withEnableFOC(false);

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

    public void rawExtension(double extension){ 
        extensionLeader.setControl(openLoop.withOutput(12 * extension));
    }

    public void setPositionControl(double position){
        // Process input
        extensionLeader.setControl(closedLoopPosition.withPosition(position));
    }

    public void setExtendingVelocity(double velocity){
        extensionLeader.setControl(closedLoopVelocity.withVelocity(velocity));
    }

    /**
     * @return The current extension of the elevator from the ground, in meters
     */
    public double getExtension(){
        return extensionLeader.getRotorPosition().getValueAsDouble() * 1.0; // TODO: rotor Position to extension m conversion
    }

    /**
     * @return The current velocity of the elevator away from the ground, in meters per second
     */
    public double getExtendingVelocity() {
        return extensionLeader.getVelocity().getValueAsDouble() * 1.0; // TODO: rotor velocity to extension m/s conversion
    }

    /**
     * Sets the elevator to maintain it's current position with a seperate PID profile
     */
    public void maintainExntension() {
        extensionLeader.setControl(closedLoopPosition.withPosition(extensionLeader.getPosition().getValueAsDouble())); 
    }

    public boolean isInwardsRotationSafe() {
        return MathUtil.isNear(
            StagingManager.StagingState.ZEROED.extension, 
            getExtension(), 
            Constants.Arm.kValidRotationTolerance);
    }

    @Override
    public void periodic() {

    }
}
