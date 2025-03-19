package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    
    private final TalonFX climbMotor = new TalonFX(Constants.IDs.kClimbMotor, Constants.kRio);
    private final PositionVoltage rotationClosedLoop = new PositionVoltage(climbMotor.getPosition().getValueAsDouble()).withEnableFOC(false);
    private final VoltageOut openLoop = new VoltageOut(0).withEnableFOC(false);

    private double initialRotation;

    // private DigitalInput maximumLimitSwitch = new DigitalInput(9);
    private DigitalInput limitSwitch = new DigitalInput(8);

    public Climber() {
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

		climbMotor.getConfigurator().apply(climbConfig);
    }

    public double getRotation(){
        return initialRotation - climbMotor.getPosition().getValueAsDouble();
    }

    public void setRotation(double position){
        position = initialRotation - position;
        climbMotor.setControl(rotationClosedLoop.withPosition(position));
    }

    public void setCoast(){
        climbMotor.setControl(openLoop.withOutput(0.0));
    }
    
    public void setInitializationRotation(){
        this.initialRotation = climbMotor.getPosition().getValueAsDouble();
    }

    public void maintainClimb(){
        climbMotor.setControl(rotationClosedLoop.withPosition(climbMotor.getPosition().getValueAsDouble()));
    }

    public SequentialCommandGroup StartupSequence(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> climbMotor.setControl(openLoop.withOutput(0.6)), this),
            new WaitUntilCommand(() -> limitSwitch.get()),
            Commands.runOnce(() -> climbMotor.setControl(openLoop.withOutput(0.0)), this)
        );
    }

    @Override
    public void periodic() {
        dashboard();
        if (limitSwitch.get()) setInitializationRotation();
    }

    private void dashboard(){
        SmartDashboard.putNumber("Climber Real", climbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber Initial Rotation", initialRotation);
        SmartDashboard.putNumber("ClimberRotationDelta", initialRotation - climbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("ClimberLimitSwitch", limitSwitch.get());
    }

}
