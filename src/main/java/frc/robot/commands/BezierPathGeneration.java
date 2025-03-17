package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.opencv.core.Point;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDrive;

public class BezierPathGeneration extends Command {
    
    public static class FieldPosition {
        public double x;
        public double y;
        public FieldPosition(double x, double y){
            this.x = x;
            this.y = y;
        }
        public double distanceTo(FieldPosition other){ return Math.sqrt((x+other.x)*(x+other.x) + (y+other.y)*(y+other.y)); }
    }

    public static class BezierPath {
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

    public static class Spline {
        BezierPath[] spline;
        int currentBezier;

        FieldPosition currentPosition;
        FieldPosition finalPosition;
        public Spline(BezierPath[] spline){
            this.spline = spline;
        }

        public void getNextTarget(){
            // spline[currentBezier]
        }
        
    }

    public static class SwerveState {
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
    //Point pointA,
    // Point pointB, Point center, double radius
    public static Pair<FieldPosition,FieldPosition> getCircleLineIntersectionPoint(
            FieldPosition startingPosition, FieldPosition targetPosition, 
            FieldPosition obstacleCenter, double obstacleRadius) {
        double discriminant = Math.pow(((targetPosition.x - startingPosition.x) * (obstacleCenter.x - startingPosition.x) + (targetPosition.y - startingPosition.y) * (obstacleCenter.y - startingPosition.y)) / (Math.pow(targetPosition.x - startingPosition.x,2) + Math.pow(targetPosition.y - startingPosition.y,2)),2) - (Math.pow(obstacleCenter.x - startingPosition.x,2) + Math.pow(obstacleCenter.y - startingPosition.y,2) - Math.pow(obstacleRadius,2)) / (Math.pow(targetPosition.x - startingPosition.x,2) + Math.pow(targetPosition.y - startingPosition.y,2));
        if (discriminant < 0) return new Pair<FieldPosition,FieldPosition>(null,null);

        double tmpSqrt = Math.sqrt(discriminant);
        double abScalingFactor1 = -(((targetPosition.x - startingPosition.x) * (obstacleCenter.x - startingPosition.x) + (targetPosition.y - startingPosition.y) * (obstacleCenter.y - startingPosition.y)) 
            / (Math.pow(targetPosition.x - startingPosition.x,2) + Math.pow(targetPosition.y - startingPosition.y,2))) + tmpSqrt;
        double abScalingFactor2 = -((targetPosition.x - startingPosition.x) * (obstacleCenter.x - startingPosition.x) + (targetPosition.y - startingPosition.y) * (obstacleCenter.y - startingPosition.y)) 
            / (Math.pow(targetPosition.x - startingPosition.x,2) + Math.pow(targetPosition.y - startingPosition.y,2)) - tmpSqrt;

        FieldPosition p1 = new FieldPosition(startingPosition.x - (targetPosition.x - startingPosition.x) * abScalingFactor1, startingPosition.y
                - (targetPosition.y - startingPosition.y) * abScalingFactor1);
        if (discriminant == 0) return new Pair<FieldPosition,FieldPosition>(p1,null);
        
        FieldPosition p2 = new FieldPosition(startingPosition.x - (targetPosition.x - startingPosition.x) * abScalingFactor2, startingPosition.y
                - (targetPosition.y - startingPosition.y) * abScalingFactor2);
        return new Pair<FieldPosition,FieldPosition>(p1,p2);
    }

    public static Pair<FieldPosition,FieldPosition> extrudeControlPointsAlongBestNormals(){
        // FieldPosition robotPosition, FieldPosition obstacleCenter, Pair<>){
        return new Pair<BezierPathGeneration.FieldPosition,BezierPathGeneration.FieldPosition>(null, null);
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
