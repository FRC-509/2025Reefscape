package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Pair;
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
            intakeMotor.getTorqueCurrent().getValueAsDouble(),
            Constants.Intake.kIntakeClockPeriod);
        this.torqueValueClock.start();
            
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
        this.intakingState = state;
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

    public void setCommandOutake(boolean commandOutaking){
        this.commandOutake = commandOutaking;
    }

    public void L4Outake(){
        intakeMotor.setControl(intakeOpenLoop.withOutput(Constants.Intake.kCoralOutakeVoltage/10));
    }

    public IntakingState getIntakingState(){
        return intakingState;
    }

    public boolean hasAlgae(){
        return intakingState.equals(IntakingState.ALGAE_INTAKE) || intakingState.equals(IntakingState.ALGAE_PASSIVE);
    }

    @Override
    public void periodic() {
        dashboard();
        
        Pair<Double,Double> stallTorqueCurrent = torqueValueClock.get(intakeMotor.getTorqueCurrent().getValueAsDouble());
        boolean atAlgaeTorque = 
            Math.abs(stallTorqueCurrent.getFirst()) > Constants.Intake.kAlgaeTorqueCurrent 
            && Math.abs(stallTorqueCurrent.getSecond()) > Constants.Intake.kAlgaeTorqueCurrent;
        
        boolean atCoralTorque = 
            Math.abs(stallTorqueCurrent.getFirst()) > Constants.Intake.kCoralTorqueCurrent 
            && Math.abs(stallTorqueCurrent.getSecond()) > Constants.Intake.kCoralTorqueCurrent;
        // Collect torque before returning command so AlternatingValueClock can update properly
        
        if(commandOutake) return; 
        if (intakingState.equals(IntakingState.ALGAE_INTAKE) && atAlgaeTorque){
            intakingState = IntakingState.ALGAE_PASSIVE;
        } else if (intakingState.equals(IntakingState.CORAL_INTAKE) && atCoralTorque) {
            intakingState = IntakingState.CORAL_PASSIVE;
        }

        if (!intakingState.equals(lastIntakingState))
            intakeMotor.setControl(intakeOpenLoop.withOutput(intakingState.voltageOut));
        
        lastIntakingState = intakingState;
    }

    public void dashboard(){
        double stallTorqueCurrent = intakeMotor.getTorqueCurrent().getValueAsDouble();
        SmartDashboard.putString("Intaking State", intakingState.toString());
        SmartDashboard.putNumber("Torque Current", stallTorqueCurrent);
        SmartDashboard.putBoolean("Has Intaken Gamepiece",
            intakingState.equals(IntakingState.ALGAE_INTAKE) && stallTorqueCurrent > Constants.Intake.kAlgaeTorqueCurrent
            || intakingState.equals(IntakingState.CORAL_INTAKE) && stallTorqueCurrent > Constants.Intake.kCoralTorqueCurrent);
        SmartDashboard.putNumber("Torque Comparison Value", torqueValueClock.timer1.get() >= torqueValueClock.timer2.get() ? torqueValueClock.first : torqueValueClock.second);
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

    public static SequentialCommandGroup outakeCommand(boolean coral, BooleanSupplier l4_Supplier, Intake intake) {
        return l4_Supplier.getAsBoolean() 
            ? new SequentialCommandGroup(
                Commands.runOnce(() -> intake.setCommandOutake(true), intake),
                Commands.runOnce(() -> intake.outake(coral), intake),
			    Commands.waitSeconds(coral ? Constants.Intake.kCoralOutakeDelay : Constants.Intake.kAlgaeOutakeDelay),
			    Commands.runOnce(() -> intake.stop(), intake),
                Commands.runOnce(() -> intake.setCommandOutake(false), intake))
            : new SequentialCommandGroup(
                Commands.runOnce(() -> intake.setCommandOutake(true), intake),
                Commands.runOnce(() -> intake.L4Outake(), intake),
			    Commands.waitSeconds(coral ? Constants.Intake.kCoralOutakeDelay : Constants.Intake.kAlgaeOutakeDelay),
			    Commands.runOnce(() -> intake.stop(), intake),
                Commands.runOnce(() -> intake.setCommandOutake(false), intake));
    }

    private class AlternatingValueCLock {
        private double period;
        private double first;
        private double second;
        private Timer timer1;
        private Timer timer2;
        private boolean firstPass;

        public AlternatingValueCLock(double initVal, double period){
            this.period = period;
            this.first = initVal;
            this.second = initVal;
            this.timer1 = new Timer();
            this.timer2 = new Timer();
            this.firstPass = true;
            start();
        }

        private void start(){
            timer1.restart();
            timer2.restart();
            firstPass = true;
        }

        private void updateFirst(double value){
            timer1.restart();
            first = value;
        }

        private void updateSecond(double value) {
            timer2.restart();
            second = value;
        }

        public Pair<Double,Double> get(double value){
            if (firstPass && timer2.get() > period/2) {
                updateSecond(value);
                firstPass = false;
            }
            if (!firstPass && Math.abs(timer1.get() - timer2.get()) < period/3) start();

            if (timer1.get() >= period) updateFirst(value);
            if (timer2.get() >= period) updateSecond(value);
            return new Pair<Double,Double>(timer1.get() >= timer2.get() ? first : second, timer1.get() < timer2.get() ? first : second);
        }
    }
}
