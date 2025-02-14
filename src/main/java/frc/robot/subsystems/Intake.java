package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    private final TalonFX intakeMotor = new TalonFX(Constants.IDs.kIntakeMotor, Constants.kCANIvore);
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
    private boolean commandOutake;
    private AlternatingValueCLock torqueValueClock;
    
    public Intake() {
        this.intakingState = IntakingState.STOP;
        this.lastIntakingState = IntakingState.STOP;
        commandOutake = false;
        this.torqueValueClock = new AlternatingValueCLock(
            () -> intakeMotor.getTorqueCurrent().getValueAsDouble(),
            Constants.Intake.kIntakeClockPeriod);
        this.torqueValueClock.start();
            
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        // Current limits
		// intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		// intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kIntakeSupply; 
        // intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kIntakeStator;

        // PID value assignment
		intakeConfig.Slot0.kP = Constants.PIDConstants.Arm.kIntakeP;
		intakeConfig.Slot0.kI = Constants.PIDConstants.Arm.kIntakeI;
		intakeConfig.Slot0.kD = Constants.PIDConstants.Arm.kIntakeD;
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		intakeMotor.getConfigurator().apply(intakeConfig);
    }

    public void setState(IntakingState state){
        // If you go from Coral Intake -> Algae Intake, it is the operator attempting to outake
        // if (state.equals(IntakingState.ALGAE_INTAKE)
        //     && (state.equals(IntakingState.CORAL_INTAKE) || state.equals(IntakingState.CORAL_PASSIVE)))
        //     this.intakingState = IntakingState.CORAL_OUTAKE;
        // else if (state.equals(IntakingState.CORAL_OUTAKE)
        //     && (state.equals(IntakingState.ALGAE_INTAKE) || state.equals(IntakingState.ALGAE_PASSIVE)))
        //     this.intakingState = IntakingState.ALGAE_OUTAKE;
        System.out.println("SETTING STATE");
        this.intakingState = state;
    }

    public void outake(boolean coral){
        intakingState = !coral ? IntakingState.CORAL_OUTAKE : IntakingState.ALGAE_OUTAKE;
        intakeMotor.setControl(intakeOpenLoop.withOutput(intakingState.voltageOut));
        lastIntakingState = intakingState;
    }

    public void stop(){
        intakingState = IntakingState.STOP;
        System.out.println("STOPPING :3");
        intakeMotor.setControl(intakeOpenLoop.withOutput(intakingState.voltageOut));
        lastIntakingState = intakingState;
    }

    public void setCommandOutake(boolean commandOutaking){
        this.commandOutake = commandOutaking;
    }

    @Override
    public void periodic() {
        dashboard();
        double stallTorqueCurrent = torqueValueClock.get();
        if(commandOutake) return; 
        if (intakingState.equals(IntakingState.ALGAE_INTAKE) && stallTorqueCurrent > Constants.Intake.kAlgaeTorqueCurrent){
            intakingState = IntakingState.ALGAE_PASSIVE;
        } else if (intakingState.equals(IntakingState.CORAL_INTAKE) 
            && stallTorqueCurrent > Constants.Intake.kCoralTorqueCurrent) {
            System.out.println("SETTING CORAL PASSIVE HERE ANYWAYS");
            intakingState = IntakingState.CORAL_PASSIVE;
        }

        if (!intakingState.equals(lastIntakingState))
            intakeMotor.setControl(intakeOpenLoop.withOutput(intakingState.voltageOut));
        
        lastIntakingState = intakingState;
    }

    public void dashboard(){
        double stallTorqueCurrent = intakeMotor.getTorqueCurrent().getValueAsDouble();
        SmartDashboard.putNumber("VoltageOutIntakeSigma", intakeMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putString("Intaking State", intakingState.toString());
        SmartDashboard.putString("Last Intaking State", lastIntakingState.toString());
        SmartDashboard.putNumber("Stall Torque Current", stallTorqueCurrent);
        SmartDashboard.putBoolean("Has Intaken Gamepiece",
            intakingState.equals(IntakingState.ALGAE_INTAKE) && stallTorqueCurrent > Constants.Intake.kAlgaeTorqueCurrent
            || intakingState.equals(IntakingState.CORAL_INTAKE) && stallTorqueCurrent > Constants.Intake.kCoralTorqueCurrent);
    }

    public static SequentialCommandGroup outakeCommand(boolean coral, Intake intake) {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> intake.setCommandOutake(true), intake),
            Commands.runOnce(() -> intake.outake(coral), intake),
			Commands.waitSeconds(coral ? Constants.Intake.kCoralOutakeDelay : Constants.Intake.kAlgaeOutakeDelay),
			Commands.runOnce(() -> intake.stop(), intake),
            Commands.runOnce(() -> intake.setCommandOutake(false), intake)
		);
    }

    private class AlternatingValueCLock {
        private double period;
        private double first;
        private double second;
        private DoubleSupplier supplier;
        private Timer timer1;
        private Timer timer2;
        private boolean firstPass;

        // NOTE: COULD run into an issue with the distance between passes drifting due to the 20ms refresh
        public AlternatingValueCLock(DoubleSupplier supplier, double period){
            this.period = period;
            this.first = supplier.getAsDouble();
            this.second = supplier.getAsDouble();
            this.timer1 = new Timer();
            this.timer2 = new Timer();
            this.firstPass = true;
        }

        private void start(){
            timer1.reset();
            timer2.reset();
            timer1.start();
            timer2.start();
            firstPass = true;
        }

        private void updateFirst(){
            timer1.reset();
            first = supplier.getAsDouble();
        }

        private void updateSecond() {
            timer2.reset();
            second = supplier.getAsDouble();
        }

        public double get(){
            if (Math.abs(timer1.get() - timer2.get()) < period/3) start(); // Possible drift midigation?
            if (firstPass && timer2.get() > period/2) {
                updateSecond();
                firstPass = false;
            }
            if (timer1.get() >= period) updateFirst();
            if (timer2.get() >= period) updateSecond();
            return timer1.get() >= timer2.get() ? first : second;
        }
    }
}
