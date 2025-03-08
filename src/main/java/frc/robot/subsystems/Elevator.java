package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.StagingManager;

public class Elevator extends SubsystemBase {
    
    private final TalonFX extensionLeader = new TalonFX(Constants.IDs.kExtensionLeader, Constants.kCANIvore);
	private final TalonFX extensionFollower = new TalonFX(Constants.IDs.kExtensionFollower, Constants.kCANIvore);

    private CANcoder extensionEncoder = new CANcoder(Constants.IDs.kExtensionEncoder,  Constants.kCANIvore);
    private CANrange rangeSensor = new CANrange(20, Constants.kCANIvore);
    private DigitalInput limitSwitch = new DigitalInput(1);

    private VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);
    private PositionDutyCycle closedLoopPosition = new PositionDutyCycle(0.0).withEnableFOC(false);

    private double initialRotation;

    public Elevator(){
        TalonFXConfiguration extensionLeaderConfig = new TalonFXConfiguration();

        // Current limits
		extensionLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		extensionLeaderConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kElevatorSupply;
        extensionLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        extensionLeaderConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kElevatorStator;

        // PID value assignment
		extensionLeaderConfig.Slot0.kP = Constants.PIDConstants.Elevator.kExtensionP;
		extensionLeaderConfig.Slot0.kI = Constants.PIDConstants.Elevator.kExtensionI;
		extensionLeaderConfig.Slot0.kD = Constants.PIDConstants.Elevator.kExtensionD;
		extensionLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Assigning encoder as feedback device
		extensionLeaderConfig.Feedback.FeedbackRemoteSensorID = extensionEncoder.getDeviceID();
		extensionLeaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		extensionLeaderConfig.Feedback.RotorToSensorRatio = 1;
        extensionLeaderConfig.Feedback.SensorToMechanismRatio = 1;
        extensionLeaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		extensionLeader.getConfigurator().apply(extensionLeaderConfig);

        TalonFXConfiguration extensionFollowerConfig = new TalonFXConfiguration();

        // Current limits
		extensionLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		extensionLeaderConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kElevatorSupply;
        extensionLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        extensionLeaderConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kElevatorStator;

        // PID value assignment
		extensionLeaderConfig.Slot0.kP = Constants.PIDConstants.Elevator.kExtensionP;
		extensionLeaderConfig.Slot0.kI = Constants.PIDConstants.Elevator.kExtensionI;
		extensionLeaderConfig.Slot0.kD = Constants.PIDConstants.Elevator.kExtensionD;
		extensionLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Assigning encoder as feedback device
		extensionLeaderConfig.Feedback.FeedbackRemoteSensorID = extensionEncoder.getDeviceID();
		extensionLeaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		extensionLeaderConfig.Feedback.RotorToSensorRatio = 1;
        extensionLeaderConfig.Feedback.SensorToMechanismRatio = 1;
        extensionLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		extensionFollower.getConfigurator().apply(extensionFollowerConfig);
		extensionFollower.setControl(new Follower(extensionLeader.getDeviceID(), true)); // TODO: Check design

		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		encoderConfig.MagnetSensor.MagnetOffset = (Constants.Elevator.kExtensionMagnetOffset) / 360.0;
		encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
		extensionEncoder.getConfigurator().apply(encoderConfig);
    }

    public void rawExtension(double voltage){ 
        extensionLeader.setControl(openLoop.withOutput(voltage));
    }

    public double getExtension(){
        return extensionLeader.getPosition().getValueAsDouble() - initialRotation;
    }

    public double getDistanceOffGround(){
        return rangeSensor.getDistance().getValueAsDouble();
    }

    public void setExtension(double extension){
        extension += initialRotation;
        SmartDashboard.putNumber("Adressed Extension", extension);
        extensionLeader.setControl(closedLoopPosition.withPosition(extension));
    }

    public boolean isInwardsRotationSafe() {
        return getExtension() < StagingManager.kRotationSafeExtension;
    }

    public void setInitializationRotation(){
        this.initialRotation = extensionLeader.getPosition().getValueAsDouble();
    }

    public void setCoast() {
        extensionLeader.setControl(openLoop.withOutput(0.0));
    }

    public boolean isZeroed() {
        return limitSwitch.get();
    }

    @Override
    public void periodic() {
        dashboard();
        if (limitSwitch.get()) setInitializationRotation();
    }

    private void dashboard(){
        SmartDashboard.putNumber("range sensor", rangeSensor.getDistance().getValueAsDouble());
        SmartDashboard.putBoolean("InwardsRotationSafe", isInwardsRotationSafe());

        SmartDashboard.putNumber("initialRotation", initialRotation);
        SmartDashboard.putNumber("ExtensionDelta", extensionLeader.getPosition().getValueAsDouble() - initialRotation);
        SmartDashboard.putNumber("GetExtension", getExtension());
        SmartDashboard.putBoolean("ElevatorLimitSwitch", limitSwitch.get());
    }

    public void setRawVoltageOut(double voltage) {
        extensionLeader.setControl(openLoop.withOutput(voltage));
    }
}
