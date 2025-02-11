package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    private final TalonFX intakeMotor = new TalonFX(Constants.IDs.kIntakeMotor);
    private final VoltageOut intakeOpenLoop = new VoltageOut(0).withEnableFOC(false);
    
    public enum IntakingState {
        CORAL_INTAKE(Constants.Intake.kCoralIntakeVoltage),
        CORAL_OUTAKE(Constants.Intake.kCoralOutakeVoltage),
        CORAL_PASSIVE(Constants.Intake.kCoralPassiveVoltage),

        ALGAE_INTAKE(Constants.Intake.kAlgaeIntakeVoltage),
        ALGAE_OUTAKE(Constants.Intake.kAlgaeOutakeVoltage),
        ALGAE_PASSIVE(Constants.Intake.kAlgaePassiveVoltage),

        STOP(0.0d);

        private double voltageOut;
        IntakingState(double voltageOut) { this.voltageOut = voltageOut; }
    }
    private IntakingState intakingState;
    private IntakingState lastIntakingState;

    public Intake() {
        this.intakingState = IntakingState.STOP;
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        // Current limits
		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kIntakeSupply; 
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kIntakeStator;

        // PID value assignment
		intakeConfig.Slot0.kP = Constants.PIDConstants.Arm.kIntakeP;
		intakeConfig.Slot0.kI = Constants.PIDConstants.Arm.kIntakeI;
		intakeConfig.Slot0.kD = Constants.PIDConstants.Arm.kIntakeD;
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		intakeMotor.getConfigurator().apply(intakeConfig);
    }

    public void setState(IntakingState state){
        // If you go from Coral Intake -> Algae Intake, it is the operator attempting to outake
        if (state.equals(IntakingState.ALGAE_INTAKE)
            && (state.equals(IntakingState.CORAL_INTAKE) || state.equals(IntakingState.CORAL_PASSIVE)))
            this.intakingState = IntakingState.CORAL_OUTAKE;
        else if (state.equals(IntakingState.CORAL_OUTAKE)
            && (state.equals(IntakingState.ALGAE_INTAKE) || state.equals(IntakingState.ALGAE_PASSIVE)))
            this.intakingState = IntakingState.ALGAE_OUTAKE;
        else this.intakingState = state;
    }

    public void outake(boolean coral){
        intakingState = coral ? IntakingState.CORAL_OUTAKE : IntakingState.ALGAE_OUTAKE;
        intakeMotor.setControl(intakeOpenLoop.withOutput(intakingState.voltageOut));
        lastIntakingState = intakingState;
    }

    public void stop(){
        intakingState = IntakingState.STOP;
        intakeMotor.setControl(intakeOpenLoop.withOutput(intakingState.voltageOut));
        lastIntakingState = intakingState;
    }

    @Override
    public void periodic() {
        double stallTorqueCurrent = intakeMotor.getTorqueCurrent().getValueAsDouble();
        if (intakingState.equals(IntakingState.ALGAE_INTAKE) && stallTorqueCurrent > Constants.Intake.kAlgaeTorqueCurrent)
            intakingState = IntakingState.ALGAE_PASSIVE;
        else if (intakingState.equals(IntakingState.ALGAE_INTAKE) && stallTorqueCurrent > Constants.Intake.kAlgaeTorqueCurrent)
            intakingState = IntakingState.CORAL_PASSIVE;

        if (!intakingState.equals(lastIntakingState))
            intakeMotor.setControl(intakeOpenLoop.withOutput(intakingState.voltageOut));
        
        lastIntakingState = intakingState;
        SmartDashboard.putString("Intaking State", intakingState.toString());
        SmartDashboard.putString("Last Intaking State", lastIntakingState.toString());
        SmartDashboard.putBoolean("Has Intaken Gamepiece",
            intakingState.equals(IntakingState.ALGAE_INTAKE) && stallTorqueCurrent > Constants.Intake.kAlgaeTorqueCurrent
            || intakingState.equals(IntakingState.ALGAE_INTAKE) && stallTorqueCurrent > Constants.Intake.kAlgaeTorqueCurrent);
    }

    public static SequentialCommandGroup outakeCommand(boolean coral, Intake intake) {
        return new SequentialCommandGroup(
			Commands.run(() -> intake.outake(coral), intake),
			Commands.waitSeconds(coral ? Constants.Intake.kCoralOutakeDelay : Constants.Intake.kAlgaeOutakeDelay),
			Commands.run(() -> intake.stop(), intake)
		);
    }
}
