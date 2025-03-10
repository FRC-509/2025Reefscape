package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Limelight;

// ALT
public class VisionAlignment {

    private Limelight limelight = new Limelight(null);

    public static enum StationOffset {

        REEF_LEFT(new Pose2d(0,0,new Rotation2d(0))), // TODO: Find Real Offsets, ensure x and y directions
        REEF_RIGHT(new Pose2d(0,0,new Rotation2d(0))),
        REEF_ALGAE(new Pose2d(0,0,new Rotation2d(0))),
        ALGAE_STATION(new Pose2d(0,0,new Rotation2d(0))),
        PROCESSOR(new Pose2d(0,0,new Rotation2d(0))),
        NO_TAG(new Pose2d(0,0,new Rotation2d(0)));

        public final Pose2d offset;

        /**
         * @param offset x and y offset in meters. x is horizontal to the front of the robot, x is horizontal offset and y is forward offset
         */
        StationOffset(Pose2d offset){
            this.offset = offset;
        }
    }

    /*
    public StationOffset getTagOffset(){
        
        switch (((int)limelight.getFiducialID())) {
            // Processor
            case 0:
                
                break;
        
            // No Tag Found
            default:
            
        }
    }

    public Translation2d FindAlignment(boolean aprilTag){
        Translation2d offset = aprilTag ? ;
        return new Translation2d();
    }
    
    public SequentialCommandGroup Processor(SwerveDrive swerve){
        Translation2d targetOrientation = FindAlignment(true);
        return new SequentialCommandGroup(
            new DefaultDriveCommand(swerve, 
                targetOrientation.getX(), 
                targetOrientation.getY(),
			    targetOrientation.getAngle().getRadians(),
                false)
        );
    }

    */
}