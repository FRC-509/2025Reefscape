package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.autonomous.Leave;
import frc.robot.autonomous.B3L1;
import frc.robot.autonomous.CL4B1;
import frc.robot.autonomous.L4;
import frc.robot.commands.*;
import frc.robot.commands.alignment.AlignmentManager;
import frc.robot.commands.alignment.AutoPickupAlgae;
import frc.robot.commands.alignment.VisionFieldAlignment;
import frc.robot.commands.staging.StagingManager;
import frc.robot.commands.staging.StagingManager.StagingState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakingState;
import frc.robot.subsystems.drive.*;
import frc.robot.util.PigeonWrapper;

import com.pathplanner.lib.auto.AutoBuilder;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick.StickButton;

public class RobotContainer {

	private final PigeonWrapper pigeon = new PigeonWrapper(0);

	private final ThrustmasterJoystick driverLeft = new ThrustmasterJoystick(0);
	private final ThrustmasterJoystick driverRight = new ThrustmasterJoystick(1);
	private final CommandXboxController operator = new CommandXboxController(2);
	private StagingManager stagingManager;
	private AlignmentManager alignmentManager;

	private final SwerveDrive swerve;
	private final Elevator elevator;
	private final Arm arm;
	private final Intake intake;
	private final Climber climber;

	public static IntakingState autoEndIntakingState;
	
	private SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		this.swerve = new SwerveDrive(pigeon);
		this.elevator = new Elevator();
		this.arm = new Arm();
		this.intake = new Intake();
		this.climber = new Climber();

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
		// // pressing down will face backward.
		(new Trigger(() -> driverRight.getPOV(0) == 0))
			.onTrue(new AlignmentManager.AlignToHeading(swerve,0));
		(new Trigger(() -> driverRight.getPOV(0) == 90))
			.onTrue(new AlignmentManager.AlignToHeading(swerve,-137));
		(new Trigger(() -> driverRight.getPOV(0) == 270))
			.onTrue(new AlignmentManager.AlignToHeading(swerve,137));
		(new Trigger(() -> driverRight.getPOV(0) == 180))
			.onTrue(new AlignmentManager.AlignToHeading(swerve,180));

		// Toggle heading correction by pressing the bottom-rightmost botton on the left
		// side of the right stick. Heading correction defaults to ON at boot.
		driverRight.isPressedBind(StickButton.LeftSideRightBottom,
				Commands.runOnce(() -> swerve.toggleHeadingCorrection(), swerve));

		// Zeroes the gyroscope when the bottom button the left stick is pressed.
		driverLeft.isPressedBind(StickButton.Left, Commands.runOnce(() -> {
			pigeon.setYaw(0);
			swerve.setTargetHeading(0);
		}, swerve));

		// driverRight.isDownBind(StickButton.Bottom,
		// 	new VisionFieldAlignment(swerve,
		// 	() -> nonInvSquare(-driverLeft.getY()),
		// 	() -> nonInvSquare(-driverLeft.getX()),
		// 	() -> nonInvSquare(-driverRight.getX())));
		
		// Auto Algae Pickup
		// driverRight.isDownBind(StickButton.Left, Commands.sequence(
		// 	StagingManager.all(StagingState.ALGAE_GROUND, elevator, arm),
		// 	new AutoPickupAlgae(
		// 		swerve, 
		// 		intake, 
		// 		() -> nonInvSquare(-driverLeft.getY()),
		// 		() -> nonInvSquare(-driverLeft.getX()),
		// 		() -> nonInvSquare(-driverRight.getX()))));
		// driverRight.isReleasedBind(StickButton.Left, 
		// 	new ConditionalCommand(
		// 		StagingManager.all(StagingState.ALGAE_SAFE, elevator, arm),
		// 		StagingManager.all(StagingState.ZEROED, elevator, arm),
		// 		() -> intake.getIntakingState().equals(IntakingState.ALGAE_PASSIVE)));


		this.alignmentManager = new AlignmentManager(
			() -> driverLeft.getJoystickButton(StickButton.Bottom).getAsBoolean(), 
			() -> driverRight.getJoystickButton(StickButton.Bottom).getAsBoolean(), 
			() -> driverRight.getJoystickButton(StickButton.Left).getAsBoolean(),
			() -> false, 
			() -> nonInvSquare(-driverLeft.getY()),
			() -> nonInvSquare(-driverLeft.getX()),
			() -> nonInvSquare(-driverRight.getX()),
			() -> nonInvSquare(-driverRight.getY()),
			swerve, elevator, arm, intake);


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
			() -> (driverLeft.getPOV(0) == 270),
			() -> (driverLeft.getPOV(0) == 90),
			() -> (driverLeft.getPOV(0) == 180),
			() -> alignmentManager.isAutoAligning());


		// // Algae Intake / outake on release
		operator.rightBumper().onTrue(Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_INTAKE), intake));
		operator.leftBumper().onTrue(Commands.runOnce(() -> intake.setState(IntakingState.CORAL_INTAKE), intake));

		operator.rightBumper().onFalse(Intake.outakeCommand(false, intake));
		operator.leftBumper().onFalse(Intake.coralConditionalOutake(intake, () -> arm.getRotation() < StagingState.ALGAE_SAFE.rotation && elevator.getExtension() > StagingState.CORAL_STATION.extension));


		// Set climber down
		operator.leftTrigger().onTrue(new SequentialCommandGroup(climber.ClimbReady(elevator, arm)));
		operator.rightTrigger().onTrue(Commands.sequence(climber.ClimbFinal(elevator, arm, swerve)));
	}

	private void addAutonomousRoutines() {
		chooser.addOption("\"Go AFK\" (Null)", new InstantCommand());
		chooser.addOption("ReverseLeave", new Leave(-0.3, 1.0, swerve));
		chooser.addOption("L4", new L4(swerve, elevator, arm, intake));
		chooser.addOption("Left B1 L1", new B3L1(swerve, elevator, arm, intake));
		chooser.addOption("Center L4 B1", new CL4B1(swerve, elevator, arm, intake));
		SmartDashboard.putData("Auto Mode", chooser);

		if (RobotBase.isSimulation()) {
			SmartDashboard.putData("Reset Swerve", Commands.runOnce(swerve::resetSimState, swerve));
		}
		autoEndIntakingState = IntakingState.STOP;
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}

	public void onRobotEnable() {
		pigeon.onEnable();
	}

	public void robotPeriodic(){
		stagingManager.update();
		alignmentManager.update();
	}

	public void onTeleopEntry() {
		stagingManager.onTeleopEntry();
	}

	public void disabledInit(){
		climber.maintainClimb();
	}
}