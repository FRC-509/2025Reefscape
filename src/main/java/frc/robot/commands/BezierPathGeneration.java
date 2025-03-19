package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.PositionAwareAlignment.TargetPosition;
import frc.robot.subsystems.drive.SwerveDrive;

public class BezierPathGeneration extends Command {
    
    public static class FieldPosition {
        public double x;
        public double y;
        public FieldPosition(double x, double y){ this.x = x; this.y = y; }
        public double distanceTo(FieldPosition other){ return Math.sqrt((x+other.x)*(x+other.x) + (y+other.y)*(y+other.y)); }
    }

    public static class Obstacle extends FieldPosition{
        double radius;
        public Obstacle(double x, double y, double radius){ super(x,y); this.radius = radius; }
    }

    public enum Location {
        
        Reef_Close_Center(new FieldPosition(14.5, 4), 0.0, new FieldPosition(0,0)),
        Reef_Close_Left(new FieldPosition(14.5, 3.87), 0.0, new FieldPosition(0,0)), // find
        Reef_Close_Right(new FieldPosition(14.5, 4.1), 0.0, new FieldPosition(0,0)), // find
        Reef_CloseRight_Center(new FieldPosition(0,0),-60, new FieldPosition(0,0)), // find
        Reef_CloseRight_Left(new FieldPosition(0,0), -60, new FieldPosition(0,0)), // find
        Reef_CloseRight_Right(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find

        Reef_CloseLeft_Center(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find
        Reef_CloseLeft_Left(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find
        Reef_CloseLeft_Right(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find

        Reef_FarRight_Center(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find
        Reef_FarRight_Left(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find
        Reef_FarRight_Right(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find

        Reef_FarLeft_Center(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find
        Reef_FarLeft_Left(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find
        Reef_FarLeft_Right(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find
        
        Reef_Far_Center(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find
        Reef_Far_Left(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find
        Reef_Far_Right(new FieldPosition(0,0),0, new FieldPosition(0,0)), // find

        CoralStation_Left(new FieldPosition(0,0),0, new FieldPosition(0,0)),
        CoralStation_Right(new FieldPosition(0,0),0, new FieldPosition(0,0)),
        
        BargeShot(new FieldPosition(0,0),0, new FieldPosition(0,0));

        public final FieldPosition position;
        public final Rotation2d rotation;
        public final Pose2d pose2d;
        public final FieldPosition headingSmoothedControlPoint;
        /**
         * @param x the x coordinate on the field
         * @param y the y coordinate on the field
         * @param headingDegrees the desired final target heading, in degrees, from 180 to -180 
         */
        Location(FieldPosition position, double headingDegrees, FieldPosition headingSmoothedControlPoint){
            this.position = position;
            this.rotation = Rotation2d.fromDegrees(headingDegrees);
            this.pose2d = new Pose2d(position.x,position.y,rotation);
            this.headingSmoothedControlPoint = headingSmoothedControlPoint;
        }
    }

    public static class BezierPath {
        public FieldPosition initialPosition;
        public double initialVelocity;

        public FieldPosition targetPosition;
        public double targetVelocity;

        public FieldPosition currentPosition;
        public double currentVelocity;

        public FieldPosition influencePointA;
        public FieldPosition influencePointB;

        public double influenceModifierA;
        public double influenceModifierB;

        public final double totalLength;
        public boolean validPath;

        public BezierPath(FieldPosition initialPosition, 
                FieldPosition targetPosition, 
                double initialVelocity, 
                double targetVelocity,
                FieldPosition influencePointA,
                FieldPosition influencePointB,
                double influenceModifierA,
                double influenceModifierB){
            this.initialPosition = initialPosition;
            this.currentPosition = initialPosition;
            this.targetPosition = targetPosition;

            this.influencePointA = influencePointA;
            this.influencePointB = influencePointB;
            this.influenceModifierA = influenceModifierA;
            this.influenceModifierB = influenceModifierB; 

            this.validPath = false;

            double sum = 0, segments = 100;
            FieldPosition last = initialPosition;
            for (int i = 0; i < segments; i++){
                FieldPosition next = get(((double)i)/segments);
                if (!isValidRobotPosition(next)) validPath = false;
                sum += last.distanceTo(next);
                last = next;
            }
            this.totalLength = sum;
        }

        public FieldPosition get(double progress){
            return new FieldPosition(
                initialPosition.x * Math.pow(1-progress, 3) 
                + influenceModifierA * influencePointA.x * progress * Math.pow(1-progress, 2)
                + influenceModifierB * influencePointB.x * Math.pow(progress, 2) * (1-progress)
                + targetPosition.x * Math.pow(progress, 3),
                initialPosition.y * Math.pow(1-progress, 3) 
                + influenceModifierA * influencePointA.y * progress * Math.pow(1-progress, 2)
                + influenceModifierB * influencePointB.y * Math.pow(progress, 2) * (1-progress)
                + targetPosition.y * Math.pow(progress, 3));
        }
    }

    public class Spline {
        BezierPath[] spline;
        int currentBezier;
        double targetHeading;
        double bezierProgress;
        double percentAcceleration;
        FieldPosition currentPosition;
        public Spline(BezierPath[] spline, double targetHeading){
            this.spline = spline;
            this.targetHeading = targetHeading;
            this.currentPosition = spline[0].initialPosition;
            this.bezierProgress = 0.0d;
        }
        public void addBezier(BezierPath bezier){
            BezierPath[] updatedSpline = new BezierPath[spline.length+1]; 
            for (int i = 0; i < spline.length; i++) updatedSpline[i] = spline[i];
            updatedSpline[updatedSpline.length-1] = bezier;
            spline = updatedSpline;
        }
        public Translation2d next(FieldPosition currentPosition){
            this.currentPosition = currentPosition;
            if (currentPosition.distanceTo(spline[spline.length-1].targetPosition) < 0.7 && percentAcceleration > 0.2) 
                percentAcceleration -= 0.05;

            double moveDistance = spline[currentBezier].totalLength 
                / (Constants.Chassis.kMaxSpeed * getVelocityAsPercentage() * 0.02);
            double distanceAsBezierPercent = moveDistance/spline[currentBezier].totalLength;
            bezierProgress += distanceAsBezierPercent;
            if (bezierProgress >= 1){
                if (currentBezier == spline.length) bezierProgress = 1;
                else {
                    bezierProgress--;
                    currentBezier++;
                }
            }
            FieldPosition target = spline[currentBezier].get(bezierProgress);
            return new Translation2d(
				MathUtil.clamp(target.x - currentPosition.x, -Constants.Chassis.kMaxSpeed, Constants.Chassis.kMaxSpeed),
				MathUtil.clamp(target.y - currentPosition.y, -Constants.Chassis.kMaxSpeed, Constants.Chassis.kMaxSpeed));
        }
    }

    public void generateSplinePath(){
        // form best case linear path
        // check if obstacles intersect that path, or safe radius
    }

    public static Pair<FieldPosition,FieldPosition> getCircleLineIntersectionPoint(
            FieldPosition startingPosition, FieldPosition targetPosition, Obstacle obstacle) {
        double discriminant = Math.pow(((targetPosition.x - startingPosition.x) * (obstacle.x - startingPosition.x) + (targetPosition.y - startingPosition.y) * (obstacle.y - startingPosition.y)) 
                / (Math.pow(targetPosition.x - startingPosition.x,2) + Math.pow(targetPosition.y - startingPosition.y,2)),2)
            - (Math.pow(obstacle.x - startingPosition.x,2) + Math.pow(obstacle.y - startingPosition.y,2) - Math.pow(obstacle.radius,2)) 
            / (Math.pow(targetPosition.x - startingPosition.x,2) + Math.pow(targetPosition.y - startingPosition.y,2));
        
        // if the discriminant is less than 0, the path does not intersect with the obstacle
        if (discriminant < 0) return new Pair<FieldPosition,FieldPosition>(null,null);

        double tmpSqrt = Math.sqrt(discriminant);
        double abScalingFactor1 = -(((targetPosition.x - startingPosition.x) * (obstacle.x - startingPosition.x) + (targetPosition.y - startingPosition.y) * (obstacle.y - startingPosition.y)) 
            / (Math.pow(targetPosition.x - startingPosition.x,2) + Math.pow(targetPosition.y - startingPosition.y,2))) + tmpSqrt;
        double abScalingFactor2 = -((targetPosition.x - startingPosition.x) * (obstacle.x - startingPosition.x) + (targetPosition.y - startingPosition.y) * (obstacle.y - startingPosition.y)) 
            / (Math.pow(targetPosition.x - startingPosition.x,2) + Math.pow(targetPosition.y - startingPosition.y,2)) - tmpSqrt;

        
        FieldPosition p1 = new FieldPosition(startingPosition.x - (targetPosition.x - startingPosition.x) * abScalingFactor1, startingPosition.y
                - (targetPosition.y - startingPosition.y) * abScalingFactor1);
        // if the discriminant is equal to 0, the path intersects with the obstacle exactly once
        if (discriminant == 0) return new Pair<FieldPosition,FieldPosition>(p1,null);
        FieldPosition p2 = new FieldPosition(startingPosition.x - (targetPosition.x - startingPosition.x) * abScalingFactor2, startingPosition.y
                - (targetPosition.y - startingPosition.y) * abScalingFactor2);
        return new Pair<FieldPosition,FieldPosition>(p1,p2);
    }

    // public static Pair<FieldPosition,FieldPosition> extrudeControlPointsAlongBestNormals(Pair<FieldPosition,FieldPosition>){
    //     // FieldPosition robotPosition, FieldPosition obstacleCenter, Pair<>){
    //     Pair<FieldPosition,FieldPosition> no
    //     return new Pair<BezierPathGeneration.FieldPosition,BezierPathGeneration.FieldPosition>(null, null);
    // }
    
    public Spline generateSpline(FieldPosition initialPosition, FieldPosition targetPosition, double targetHeading, Obstacle... obstacles){
        BezierPath[] bezierPath = new BezierPath[obstacles.length];
        if (obstacles.length == 0)
            return new Spline(
                new BezierPath[]{
                    new BezierPath(
                        initialPosition, 
                        targetPosition, 
                        getVelocityAsPercentage(), 
                        0.0d, 
                        initialPosition, targetPosition,
                        3, 3)
                },
                targetHeading); // straight line adjustment
        for (int i = 1; i < obstacles.length; i++){
            // Pair<FieldPosition,FieldPosition> centeredPathIntersectionPoints = getCircleLineIntersectionPoint(initialPosition, targetPosition, obstacles[i]);
            // Pair<FieldPosition,FieldPosition> leftBoundPathIntersectionPoints = getCircleLineIntersectionPoint(initialPosition, targetPosition, obstacles[i]);
            // Pair<FieldPosition,FieldPosition> rightBoundPathIntersectionPoints = getCircleLineIntersectionPoint(initialPosition, targetPosition, obstacles[i]);
            
            // // for all of them, check to see if they are 
        
        
        }

        return new Spline(bezierPath,targetHeading);
    }

    public Obstacle[] defineObstacles(){
        

        return new Obstacle[0];
    }

    public static boolean isValidRobotPosition(FieldPosition position){
        return position.x + (Constants.Chassis.kRobotWidth + Constants.Chassis.kSafePathingTolerance) < Constants.Field.kFullFieldLength 
            && position.x - (Constants.Chassis.kRobotWidth + Constants.Chassis.kSafePathingTolerance) > 0
            && position.y + (Constants.Chassis.kRobotWidth + Constants.Chassis.kSafePathingTolerance) < Constants.Field.kFieldWidth 
            && position.y - (Constants.Chassis.kRobotWidth + Constants.Chassis.kSafePathingTolerance) > 0;
    }

    public FieldPosition estimatedCurrentPosition(){
        Pose2d estimatedPose = swerve.getEstimatedPose();
        return new FieldPosition(estimatedPose.getX(), estimatedPose.getY());
    }

    public double getVelocityAsPercentage(){
        ChassisSpeeds speeds = swerve.getChassisSpeeds();
        return Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond); 
    }
    
    private SwerveDrive swerve;
    private Field2d field;
    private Spline spline;

    private FieldPosition startingPosition;
    private FieldPosition goalPosition;

    private double targetHeading;

    public BezierPathGeneration(FieldPosition goalPosition, double targetHeading, SwerveDrive swerve){
        this.swerve = swerve;
        this.startingPosition = estimatedCurrentPosition();
        this.goalPosition = goalPosition;
        this.targetHeading = targetHeading;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.spline = generateSpline(goalPosition, goalPosition, targetHeading, defineObstacles());
    }

    @Override
    public void execute() {
        swerve.drive(
            spline.next(estimatedCurrentPosition()), 
            0,
            true, 
            false);
    }
    
    @Override
    public boolean isFinished() {
        return estimatedCurrentPosition().distanceTo(goalPosition) < Constants.Chassis.kValidPositionTolerance
            && MathUtil.isNear(swerve.getYaw().getDegrees(), targetHeading, Constants.Chassis.kValidHeadingTolerance);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
