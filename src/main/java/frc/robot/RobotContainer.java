package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.autonomous.Actions;
import frc.robot.autonomous.Actions.PathValidation;
import frc.robot.autonomous.Leave;
import frc.robot.autonomous.Test;
import frc.robot.commands.*;
import frc.robot.commands.StagingManager.StagingState;
import frc.robot.commands.staging.ExtendTo;
import frc.robot.commands.staging.RotateTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakingState;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.PigeonWrapper;
import frc.robot.util.ThinNT;

import com.redstorm509.stormkit.controllers.ThrustmasterJoystick;
import com.redstorm509.stormkit.controllers.ThrustmasterJoystick.StickButton;

public class RobotContainer {

	private final PigeonWrapper pigeon = new PigeonWrapper(0);

	private final ThrustmasterJoystick driverLeft = new ThrustmasterJoystick(0);
	private final ThrustmasterJoystick driverRight = new ThrustmasterJoystick(1);
	private final CommandXboxController operator = new CommandXboxController(2);
	public StagingManager stagingManager;
	public AlignmentManager alignmentManager;

	private final SwerveDrive swerve;
	private final Elevator elevator;
	private final Arm arm;
	private final Intake intake;
	private final Climber climber;
	
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
				() -> false,
				() -> true));

		// Binds heading locks to the right stick's dpad. Pressing up will face forward,
		// // pressing down will face backward.
		(new Trigger(() -> driverRight.getPOV(0) == 0))
			.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(0), swerve));
		(new Trigger(() -> driverRight.getPOV(0) == 90))
			.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(-137), swerve));
		(new Trigger(() -> driverRight.getPOV(0) == 270))
			.onTrue(Commands.runOnce(() -> swerve.setTargetHeading(137), swerve));
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
			// () -> driverLeft.getJoystickButton(StickButton.Bottom).getAsBoolean(),
			() -> (driverLeft.getPOV(0) == 0),
			() -> (driverLeft.getPOV(0) == 270),
			() -> (driverLeft.getPOV(0) == 90),
			() -> (driverLeft.getPOV(0) == 180));

		// this.alignmentManager = new AlignmentManager(
		// 	() -> nonInvSquare(-driverLeft.getY()), 
		// 	() -> driverRight.getPOV(0) == 90, 
		// 	() -> driverRight.getPOV(0) == 270,
		// 	() -> driverRight.getPOV(0) == 180,
		// 	swerve, elevator, arm, intake);

		// // Algae Intake / outake on release
		operator.rightBumper().onTrue(Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_INTAKE), intake));
		operator.leftBumper().onTrue(Commands.runOnce(() -> intake.setState(IntakingState.CORAL_INTAKE), intake));

		operator.rightBumper().onFalse(Intake.outakeCommand(false, intake));
		operator.leftBumper().onFalse(Intake.outakeCommand(true, intake));

		driverLeft.getJoystickButton(StickButton.Bottom).onTrue(new DriveToLocation(swerve));


		// Set climber down
		operator.leftTrigger().onTrue(new SequentialCommandGroup(
			Commands.runOnce(() -> arm.setRotation(0.43), arm),
			Commands.waitSeconds(0.55),
			Commands.runOnce(() -> elevator.setExtension(StagingState.CORAL_STATION.extension), elevator),
			climber.StartupSequence(),
			Commands.runOnce(() -> climber.setRotation(Constants.Climber.kClimbReadyPosition), climber)
		));
		operator.rightTrigger().onTrue(Commands.sequence(
			Commands.runOnce(() -> climber.setRotation(Constants.Climber.kClimbFinalPosition), climber),
			Commands.parallel(
				Commands.waitSeconds(10),
				Commands.run(() -> swerve.drive(new Translation2d(0,0), 0, true, true), swerve)
			))
		);
	}

	private void addAutonomousRoutines() {
		chooser.addOption("\"Go AFK\" (Null)", new InstantCommand());
		chooser.addOption("Leave", new Leave(0.3, 1.0, swerve));
		chooser.addOption("Test", Commands.sequence(
			Commands.runOnce(() -> intake.setState(IntakingState.CORAL_PASSIVE), intake),
			Commands.runOnce(() ->swerve.setChassisSpeeds(new ChassisSpeeds(Constants.Chassis.kMaxSpeed * 0.2, 0, 0)),swerve),
            Commands.waitSeconds(1),
            Commands.runOnce(() ->swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0)),swerve),
			StagingManager.L4_Rising(elevator, arm),
			Commands.waitSeconds(0.2),
			Commands.runOnce(() ->swerve.setChassisSpeeds(new ChassisSpeeds(Constants.Chassis.kMaxSpeed * 0.2, 0, 0)),swerve),
            Commands.waitSeconds(1),
            Commands.runOnce(() ->swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0)),swerve),
			StagingManager.L4_Falling(elevator, arm, intake, () -> true),
			Commands.runOnce(() -> elevator.setExtension(3.85), elevator),
            Commands.runOnce(() -> arm.setRawVoltageOut(-0.2), arm),
            Commands.runOnce(() -> intake.L4Outake()),
			Commands.runOnce(() ->swerve.setChassisSpeeds(new ChassisSpeeds(Constants.Chassis.kMaxSpeed * 0.2, 0, 0)),swerve),
            new WaitUntilCommand(() -> (arm.getRotation() > 0.22418)),
			Commands.waitSeconds(0.2),
            Commands.runOnce(() ->swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0)),swerve),
			StagingManager.zero(elevator, arm, intake)
		));
		chooser.addOption("PP", Commands.sequence(

		));
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
		SmartDashboard.putNumberArray("OutputMove", new double[]{LimelightHelpers.getBotPose3d_TargetSpace(Constants.Vision.rightLimelight).getX(),LimelightHelpers.getBotPose3d_TargetSpace(Constants.Vision.rightLimelight).getY()});

	}

	public void onTeleopEntry() {
		pigeon.setYaw(180); // temp to correct yaw after straight leave auto
	}

	public void disabledInit(){
		climber.maintainClimb();
	}
}