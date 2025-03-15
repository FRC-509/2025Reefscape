package frc.robot.commands;

import java.security.PublicKey;
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
        // double a = d.Dot(d);
        // double b = 2*f.Dot(d);
        // double c = f.Dot(f) - r*r;
        
        // double discriminant = b*b-4*a*c;
        // if( discriminant < 0 ){
        //     // no intersection
        // } else {
        //     // ray didn't totally miss sphere,
        //     // so there is a solution to
        //     // the equation.
  
        //     discriminant = Math.sqrt(discriminant);

        //     // either solution may be on or off the ray so need to test both
        //     // t1 is always the smaller value, because BOTH discriminant and
        //     // a are nonnegative.
        //     double t1 = (-b - discriminant)/(2*a);
        //     double t2 = (-b + discriminant)/(2*a);

        //     // 3x HIT cases:
        //     //          -o->             --|-->  |            |  --|->
        //     // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit), 
        
        //     // 3x MISS cases:
        //     //       ->  o                     o ->              | -> |
        //     // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)
  
        //     if( t1 >= 0 && t1 <= 1 ){
        //         // t1 is the intersection, and it's closer than t2
        //         // (since t1 uses -b - discriminant)
        //         // Impale, Poke
        //         return true;
        //     }

        //     // here t1 didn't intersect so we are either started
        //     // inside the sphere or completely past it
        //     if( t2 >= 0 && t2 <= 1 ){
        //         // ExitWound
        //         return true;
        //     }
        //     // no intn: FallShort, Past, CompletelyInside
        //     return false;
        // }
        return false;
    }
    
    public void addBezier(BezierPath bezier, ArrayList<BezierPath> spline, int index){
        
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
