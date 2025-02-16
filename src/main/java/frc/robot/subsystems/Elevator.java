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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.StagingManager;
import frc.robot.commands.StagingManager.StagingState;

public class Elevator extends SubsystemBase {
    
    private final TalonFX extensionLeader = new TalonFX(Constants.IDs.kExtensionLeader, Constants.kCANIvore); // TODO: ID ME :(
	private final TalonFX extensionFollower = new TalonFX(Constants.IDs.kExtensionFollower, Constants.kCANIvore);

    private CANcoder extensionEncoder = new CANcoder(Constants.IDs.kExtensionEncoder,  Constants.kCANIvore);
    private CANrange rangeSensor = new CANrange(20, Constants.kCANIvore);

    private VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);
    private PositionDutyCycle closedLoopPosition = new PositionDutyCycle(0.0).withEnableFOC(false);
	// private PositionVoltage closedLoopPosition = new PositionVoltage(0).withEnableFOC(false);
    // private VelocityVoltage closedLoopVelocity = new VelocityVoltage(0).withEnableFOC(false);

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

    public void rawExtension(double extension){ 
        extensionLeader.setControl(openLoop.withOutput(12 * extension));
    }

    public void setPositionControl(double position){
        // Process input
        extensionLeader.setControl(closedLoopPosition.withPosition(position));
    }

    public void setExtendingVelocity(double velocity){
        //extensionLeader.setControl(closedLoopVelocity.withVelocity(velocity));
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

    public void setExtension(double state){
        SmartDashboard.putNumber("Adressed Extension", state);
        extensionLeader.setControl(closedLoopPosition.withPosition(state));
    }

    public boolean isInwardsRotationSafe() {
        return getExtension() < 0.138;
    }

    public void extendTo(double rotationPosition){
        extensionLeader.setControl(closedLoopPosition.withPosition(rotationPosition));
    }


    @Override
    public void periodic() {
        dashboard();
        // extendTo(SmartDashboard.getNumber("ExtendTo", extensionLeader.getRotorPosition().getValueAsDouble()));
        // SmartDashboard.putNumber("", extensionFollower.getPos);
    }

    private void dashboard(){
        SmartDashboard.putBoolean("InwardsRotationSafe", isInwardsRotationSafe());
        // SmartDashboard.putNumber("Leader Extension Position", extensionLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Extension Position", extensionLeader.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("FollowerExtensionPosition", extensionFollower.getPosition().getValueAsDouble());
    }
}
