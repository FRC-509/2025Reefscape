package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.autonomous.Leave;
import frc.robot.commands.*;
import frc.robot.commands.StagingManager.StagingState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakingState;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.PigeonWrapper;

import com.redstorm509.stormkit.controllers.ThrustmasterJoystick;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick.StickButton;

public class RobotContainer {

	private final PigeonWrapper pigeon = new PigeonWrapper(0);
	// private final Limelight baseLimelight = new Limelight(null);
	// private final Limelight highLimelight = new Limelight(null);

	private final ThrustmasterJoystick driverLeft = new ThrustmasterJoystick(0);
	private final ThrustmasterJoystick driverRight = new ThrustmasterJoystick(1);
	private final CommandXboxController operator = new CommandXboxController(2);
	public StagingManager stagingManager;

	private final SwerveDrive swerve;
	private final Elevator elevator;
	private final Arm arm;
	private final Intake intake;
	// private final Climber climber;
	
	private SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		this.swerve = new SwerveDrive(pigeon, new Limelight("womp womp"));
		this.elevator = new Elevator();
		this.arm = new Arm();
		this.intake = new Intake();
		// this.climber = new Climber();

		configureButtonBindings();
		addAutonomousRoutines();
	}

	private static double nonInvSquare(double axis) {
		double deadbanded = MathUtil.applyDeadband(axis, Constants.Operator.kStickDeadband);
		double squared = Math.abs(deadbanded) * deadbanded;
		return squared;
	}

	private void configureButtonBindings() {

		// DRIVER ------------------------------------

		// Binds translation to the left stick, and rotation to the right stick.
		// Defaults to field-oriented drive unless the left button on the left stick is
		// held down.

		swerve.setDefaultCommand(new DefaultDriveCommand(
				swerve,
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> nonInvSquare(-driverRight.getX()),
				() -> driverLeft.getTrigger(),
				() -> driverRight.getTrigger(),
				() -> true));

		// Binds heading locks to the right stick's dpad. Pressing up will face forward,
		// pressing down will face backward.
		(new Trigger(() -> driverRight.getPOV(0) == 0))
			.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(0), swerve));
		(new Trigger(() -> driverRight.getPOV(0) == 90))
			.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(-90), swerve));
		(new Trigger(() -> driverRight.getPOV(0) == 270))
			.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(90), swerve));
		(new Trigger(() -> driverRight.getPOV(0) == 180))
			.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(180), swerve));

		// Toggle heading correction by pressing the bottom-rightmost botton on the left
		// side of the right stick. Heading correction defaults to ON at boot.
		driverRight.isPressedBind(StickButton.LeftSideRightBottom,
				Commands.runOnce(() -> swerve.toggleHeadingCorrection(), swerve));

		// Zeroes the gyroscope when the bottom button the left stick is pressed.
		driverLeft.isPressedBind(StickButton.Left, Commands.runOnce(() -> {
			pigeon.setYaw(0);
			swerve.setTargetHeading(0);
		}, swerve));


		// OPERATOR ------------------------------------

		this.stagingManager = new StagingManager(
			elevator, 
			arm, 
			intake, 
			() -> operator.a().getAsBoolean(),
			() -> operator.b().getAsBoolean(),
			() -> operator.y().getAsBoolean(),
			() -> operator.x().getAsBoolean(),
			() -> driverLeft.getJoystickButton(StickButton.Right).getAsBoolean(),
			() -> driverRight.getJoystickButton(StickButton.Right).getAsBoolean(),
			() -> driverRight.getJoystickButton(StickButton.Left).getAsBoolean(),
			() -> (driverLeft.getPOV(0) == 0),
			() -> (driverLeft.getPOV(0) == 90),
			() -> (driverLeft.getPOV(0) == 180));

		// (new Trigger(() -> driverLeft.getPOV(0) == 0))
		// 	.onTrue(StagingManager.zero(elevator, arm, intake));

		// (new Trigger(() -> driverLeft.getPOV(0) == 90))
		// 	.onTrue(StagingManager.softResetSuperstructure(elevator, arm));
		// (new Trigger(() -> driverLeft.getPOV(0) == 180))
		// 	.onTrue(StagingManager.hardstopSuperstructure(elevator, arm));

		
		// Elevator / Arm setpoint control

		// operator.a().onTrue(StagingManager.L4_Rising(elevator, arm));
		// operator.b().onTrue(StagingManager.all(StagingState.CORAL_L3, elevator, arm));
		// operator.y().onTrue(StagingManager.all(StagingState.CORAL_L2, elevator, arm));
		// operator.x().onTrue(StagingManager.all(StagingState.CORAL_L1, elevator, arm));
		
		// operator.a().onFalse(StagingManager.L4_Falling(elevator, arm, intake));
		// operator.x().onFalse(StagingManager.zero(elevator, arm, intake));
		// operator.y().onFalse(StagingManager.zero(elevator, arm, intake));
		// operator.b().onFalse(StagingManager.zero(elevator, arm, intake));

		// // Algae Intake / outake on release
		operator.rightBumper().onTrue(Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_INTAKE), intake));
		operator.leftBumper().onTrue(Commands.runOnce(() -> intake.setState(IntakingState.CORAL_INTAKE), intake));

		operator.rightBumper().onFalse(Intake.outakeCommand(false, intake));
		operator.leftBumper().onFalse(Intake.outakeCommand(true, intake));

		// climber.setDefaultCommand(new DefaultClimbCommand(
		// 	() -> operator.getRightY(), 
		// 	() -> operator.getRightTriggerAxis() >= Constants.Operator.kTriggerDeadband, 
		// 	climber));
	}

	private void addAutonomousRoutines() {
		chooser.addOption("\"Go AFK\" (Null)", new InstantCommand());
		chooser.addOption("Leave", new Leave(0.3, 1.0, swerve));
		SmartDashboard.putData("Auto Mode", chooser);

		if (RobotBase.isSimulation()) {
			SmartDashboard.putData("Reset Swerve", Commands.runOnce(swerve::resetSimState, swerve));
		}
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}

	public void onRobotEnable() {
		pigeon.onEnable();
	}

	public void robotPeriodic(){
		stagingManager.update();
	}

	public void onTeleopEntry() {
	}
}