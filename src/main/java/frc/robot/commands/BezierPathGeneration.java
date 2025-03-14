package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDrive;

public class BezierPathGeneration extends Command {
    
    public class FieldPosition {
        public double x;
        public double y;
        public FieldPosition(double x, double y){
            this.x = x;
            this.y = y;
        }
        public double distanceTo(FieldPosition other){ return Math.sqrt((x+other.x)*(x+other.x) + (y+other.y)*(y+other.y)); }
    }

    public class BezierPath {
        public FieldPosition initialPosition;
        public SwerveState initialState;

        public FieldPosition targetPosition;
        public SwerveState targetState;

        public FieldPosition currentPosition;
        public SwerveState currentState;

        public FieldPosition influencePointA;
        public FieldPosition influencePointB;

        public BezierPath(FieldPosition initialPosition, 
                SwerveState initialState,
                FieldPosition targetPosition,
                SwerveState targetState){
            this.initialPosition = initialPosition;
            this.initialState = initialState;

            this.currentPosition = initialPosition;
            this.targetPosition = targetPosition;
        }

        public SwerveState nextSwerveState(){
            return new SwerveState(currentState.velocity, currentState.acceleration, currentState.yaw);
        }
    }

    public class SwerveState {
        public double velocity;
        public double acceleration;
        public double yaw;
        public SwerveState(double velocity, double acceleration, double yaw){
            update(velocity, acceleration, yaw);
        }
        public void update(double velocity, double acceleration, double yaw){
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.yaw = yaw;
        }
    }

    public void generateSplinePath(){
        // form best case linear path
        // check if obstacles intersect that path, or safe radius
    }

    public boolean checkForIntersection(FieldPosition robotPosition, double pathSlope, FieldPosition obstaclePosition, double obstacleRadius){
        
        return false;
    }
    
    
    
    private SwerveDrive swerve;
    private Field2d field;
    private ArrayList<BezierPath> splinePath;

    public BezierPathGeneration(SwerveDrive drive){
        field = swerve.getField();
        splinePath = new ArrayList<BezierPath>();
        
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        field = swerve.getField();
        
        swerve.drive(null, 0, isFinished(), isScheduled());
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
