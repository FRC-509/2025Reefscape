package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Limelight;

// ALT
public class VisionAlignment {

    private Limelight limelight = new Limelight(null);

    public static enum StationOffset {

        REEF_LEFT(new Translation2d(0,0)), // TODO: Find Real Offsets, ensure x and y directions
        REEF_RIGHT(new Translation2d(0,0)),
        REEF_ALGAE(new Translation2d(0,0)),
        ALGAE_STATION(new Translation2d(0,0)),
        PROCESSOR(new Translation2d(0,0)),
        NO_TAG(new Translation2d());

        public final Translation2d offset;

        /**
         * @param offset x and y offset in meters. x is horizontal to the front of the robot, x is horizontal offset and y is forward offset
         */
        StationOffset(Translation2d offset){
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