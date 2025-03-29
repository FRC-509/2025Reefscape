package frc.robot.commands.alignment;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.staging.StagingManager;
import frc.robot.commands.staging.StagingManager.StagingState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakingState;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AlignmentManager {

    private class AlignmentTrigger {
        BooleanSupplier booleanSupplier;
        boolean last; 
        Runnable onTrue; 
        Runnable onFalse;
        BooleanSupplier alternateCondition;

        public AlignmentTrigger(
            BooleanSupplier booleanSupplier, 
            Runnable onTrue,
            Runnable onFalse){
            this.booleanSupplier = booleanSupplier;
            this.last = false;
            this.onTrue = onTrue;
            this.onFalse = onFalse;
        }
    }

    private void onChange(AlignmentTrigger trigger) {
        if (!trigger.last && trigger.booleanSupplier.getAsBoolean())
            (trigger.alternateCondition.getAsBoolean() ?  trigger.onTrue : trigger.onFalse).run();
        trigger.last = trigger.booleanSupplier.getAsBoolean();
    }

    private SwerveDrive swerve;
    private Elevator elevator;
    private Arm arm;
    private Intake intake;

    private BooleanSupplier alignmentTrigger;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier; 
    private DoubleSupplier rotSupplier;
    private DoubleSupplier driverRightAlt;
    
    private AlignmentTrigger fieldElement;
    private AlignmentTrigger algaeGround;
    private AlignmentTrigger coralGround;

    private Command alignmentCommand;

    public AlignmentManager(
        BooleanSupplier alignmentTrigger,
        BooleanSupplier fieldElementSupplier,
        BooleanSupplier algaeGroundSupplier,
        BooleanSupplier coralGroundSupplier,
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier rotSupplier,
        DoubleSupplier driverRightAlt,
        SwerveDrive swerve,
        Elevator elevator,
        Arm arm,
        Intake intake
    ){
        this.swerve = swerve;
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;

        this.alignmentTrigger = alignmentTrigger;
        
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier; 
        this.rotSupplier = rotSupplier;
        this.driverRightAlt = driverRightAlt;

        this.fieldElement = new AlignmentTrigger(
            fieldElementSupplier,
            () -> {
                LimelightHelpers.setPipelineIndex(Constants.Vision.leftLimelight, Constants.Vision.Pipeline.AprilTags);
		        LimelightHelpers.setPipelineIndex(Constants.Vision.rightLimelight, Constants.Vision.Pipeline.AprilTags);
		        
                String activeLimelight;
                if (!LimelightHelpers.getTV(Constants.Vision.leftLimelight) && !LimelightHelpers.getTV(Constants.Vision.rightLimelight)) return;

		        // If both limelights see a target, prioritize the limelight with the tag taking up the larger percentage camera space 
		        if (LimelightHelpers.getTV(Constants.Vision.leftLimelight) && LimelightHelpers.getTV(Constants.Vision.rightLimelight))
		        	activeLimelight = LimelightHelpers.getTA(Constants.Vision.leftLimelight) < LimelightHelpers.getTA(Constants.Vision.rightLimelight) 
		        		? Constants.Vision.rightLimelight : Constants.Vision.leftLimelight;
		        else activeLimelight = LimelightHelpers.getTV(Constants.Vision.rightLimelight) ? Constants.Vision.rightLimelight : Constants.Vision.leftLimelight; // Otherwise whichever has tag
            
                alignmentCommand = getBehavior((int)LimelightHelpers.getFiducialID(activeLimelight));
                alignmentCommand.schedule();
            },
            () -> { alignmentCommand.cancel(); });

        this.algaeGround = new AlignmentTrigger(
            algaeGroundSupplier,
            () -> new AutoPickupAlgae(swerve, intake, xSupplier, ySupplier, rotSupplier),
            () -> StagingManager.zero(elevator, arm, intake).schedule());
    
        this.coralGround = new AlignmentTrigger(
            coralGroundSupplier,
            () -> StagingManager.all(StagingState.CORAL_GROUND, elevator, arm).schedule(), 
            () -> StagingManager.zero(elevator, arm, intake).schedule());
    }

    public void update(){
        if (!alignmentTrigger.getAsBoolean()) return;
        onChange(fieldElement);
        onChange(algaeGround);
        onChange(coralGround);
    }

    private Command getBehavior(int tagID){
		switch (tagID) {
			// Coral Station
			case 1: case 13: return Commands.sequence(
                    new AlignToHeading(swerve, 137),
                    new AlignTo(swerve, Constants.Vision.leftLimelight));
			case 2: case 12: return Commands.sequence(
                new AlignToHeading(swerve, 137),
                new AlignTo(swerve, Constants.Vision.leftLimelight));

			// Reef
			case 9: case 22: return reefAutoBehaivor(-137);
			case 11: case 20: return reefAutoBehaivor(137);
			case 6: case 19: return reefAutoBehaivor(47);
			case 8: case 17: return reefAutoBehaivor(-47);
			case 7: case 18: return reefAutoBehaivor(0);
			case 10: case 21: return reefAutoBehaivor(180);
			// Barge
			case 5: case 14: return Commands.sequence(
                    new AlignToHeading(swerve, 137),
                    new AlignTo(swerve, Constants.Vision.leftLimelight),
                    new WaitUntilCommand(() -> driverRightAlt.getAsDouble() > 0.6),
                    StagingManager.L4_Rising(elevator, arm, intake, () -> true),
                    Commands.waitSeconds(0.6),
                    Commands.parallel(
                        Commands.sequence(
                            Commands.waitSeconds(0.5),
                            Commands.runOnce(() -> intake.setState(IntakingState.ALGAE_OUTAKE)),
                            Commands.waitSeconds(Constants.Intake.kAlgaeOutakeDelay),
                            Commands.runOnce(() -> intake.stop())
                        ),
                        Commands.deadline(
                            Commands.waitSeconds(1),
                            Commands.runEnd(
                                () -> swerve.setChassisSpeeds(new ChassisSpeeds(
                                    0.2 * Constants.Chassis.kMaxSpeed,
                                    ySupplier.getAsDouble() * Constants.Chassis.kMaxSpeed * 0.3,
                                    0)), 
                                () -> {
                                    swerve.setChassisSpeeds(new ChassisSpeeds());
                                    StagingManager.L4_Falling(elevator, arm, intake, () -> false).schedule(); // assert that this doesn't interrupt the whole command
                                }, swerve)
                        )
                    )
                );

			default: 
				return Commands.none();
		}
	}

    private Command reefAutoBehaivor(double heading){
        return Commands.sequence(
            new AlignToHeading(swerve, 0),
            new AlignTo(swerve, Constants.Vision.rightLimelight),
            new WaitUntilCommand(() -> rotSupplier.getAsDouble() > 0.6 || driverRightAlt.getAsDouble() > 0.6),
            new ConditionalCommand(
                Commands.sequence(
                    driveFor(0.2, 0, 0.75),
                    new WaitUntilCommand(() -> driverRightAlt.getAsDouble() < 0.6),
                    driveFor(0.2, 0, 0.75)
                ),
                Commands.sequence(
                    driveFor(0, 0.2 * (driverRightAlt.getAsDouble()/Math.abs(driverRightAlt.getAsDouble())), 0.2), // find adjustment time
                    new WaitUntilCommand(() -> driverRightAlt.getAsDouble() < 0.6),
                    driveFor(0.1, 0, 0.3)                
                ), 
            () -> driverRightAlt.getAsDouble() > 0.6)
        );
    }

    private class AlignToHeading extends Command {
        SwerveDrive swerve;
        double heading;
        TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(Constants.Chassis.kMaxAngularVelocity/1.5, Math.PI/2));
        AlignToHeading(SwerveDrive swerve, double heading){
            this.swerve = swerve;
            this.heading = heading;
        }
        @Override
        public void execute() {
            swerve.drive(
                new Translation2d(), 
                trapezoidProfile.calculate(0.02, 
                    new State(
                        swerve.getYaw().getDegrees(), 
                        swerve.getChassisSpeeds().omegaRadiansPerSecond), 
                    new State(heading,0))
                .velocity,
                false, 
                false);
        }
        @Override
        public boolean isFinished() {
            return MathUtil.isNear(heading, swerve.getYaw().getDegrees(), 0.75);
        }
        @Override
        public void end(boolean interrupted) {
            swerve.setTargetHeading(swerve.getYaw().getDegrees());
            swerve.drive(new Translation2d(), 0, true, false);
        }
    }

    private class AlignTo extends Command {

	    private SwerveDrive swerve;
	    private String limelight;
        private Translation2d outputTranslation;

	    private static final double kReefAlignedTagPercent = 2.33;
	    private static final double ktP = 0.5;

	    public AlignTo(SwerveDrive swerve, String limelight) {
	    	this.swerve = swerve;
            this.limelight = limelight;
		    addRequirements(swerve);
	    }

    	@Override
	    public void execute() {
            outputTranslation = limelight.equals(Constants.Vision.rightLimelight) 
                ? new Translation2d(
                    MathUtil.clamp((kReefAlignedTagPercent - LimelightHelpers.getTA(limelight))/kReefAlignedTagPercent,
                        -Constants.Chassis.kMaxSpeed,
                        Constants.Chassis.kMaxSpeed),
                    -LimelightHelpers.getTX(limelight)/(Constants.Vision.kxFOV) * 0.5)
                : new Translation2d(
		    		LimelightHelpers.getTY(limelight)/(Constants.Vision.kyFOV),
	    			LimelightHelpers.getTX(limelight)/(Constants.Vision.kxFOV));
		    swerve.drive(outputTranslation.times(Constants.Chassis.kMaxSpeed * ktP), 
                0, false, false);
        }

        @Override
        public boolean isFinished() {
            return outputTranslation.getDistance(Translation2d.kZero) < 0.25;
        }

	    @Override
	    public void end(boolean wasInterrupted) {
	    	swerve.drive(new Translation2d(0, 0), 0, true, false);
	    	swerve.setTargetHeading(swerve.getYaw().getDegrees());
	    }
    }

    public Command driveFor(double xSpeed, double ySpeed, double seconds){
        return Commands.deadline(
            Commands.waitSeconds(seconds),
            Commands.runEnd(() -> swerve.setChassisSpeeds(new ChassisSpeeds(
                xSpeed * Constants.Chassis.kMaxSpeed,
                ySpeed * Constants.Chassis.kMaxSpeed,
            0)), null, swerve));
    }

    public boolean isAutoAligning(){
        return alignmentTrigger.getAsBoolean();
    }    
}
