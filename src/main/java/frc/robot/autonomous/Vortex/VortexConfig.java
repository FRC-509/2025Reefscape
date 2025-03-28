package frc.robot.autonomous.Vortex;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.FieldPosition;
import java.util.ArrayList;
import java.util.Collection;
import java.util.function.BooleanSupplier;
import java.util.function.ObjDoubleConsumer;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.SwerveDrive;

public class VortexConfig {

    public static interface PathController {
        void robotRelativeDrive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond);
        void fieldRelativeDrive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond);
    }

    public static class Action {
        public final String name;
        public final Command action;
        public Action(String name, Command action){ this.name = name; this.action = action; }
    }

    public static class ConditionalTrigger {
        public final String name;
        public final BooleanSupplier condition;
        public final boolean waitUntil;
        public ConditionalTrigger(String name, BooleanSupplier condition, boolean waitUntil){
            this.name = name;
            this.condition = condition;
            this.waitUntil = waitUntil;
        }
    }

    public final double robotWeight; 
    public final double robotMOI;
    public final double robotLengh;
    public final double robotWidth;

    public final PathController pathController;

    @SuppressWarnings("unchecked")
    public VortexConfig(
            double robotWeight,
            double robotMOI,
            double robotLengh,
            double robotWidth, 
            PathController pathController, 
            Action[] actions, 
            ConditionalTrigger[] conditionalTriggers){
        this.robotWeight = robotWeight;
        this.robotMOI = robotMOI;
        this.robotLengh = robotLengh;
        this.robotWidth = robotWidth;
        this.pathController = pathController;

        JSONObject config = new JSONObject();
        config.put("length", robotLengh);
        config.put("width", robotWidth);

        JSONArray actionArray = new JSONArray();
        for (Action action : actions) actionArray.add(action.name);
        config.put("actions", actionArray);

        JSONArray conditionalTriggersArray = new JSONArray();
        for (ConditionalTrigger conditionalTrigger : conditionalTriggers) conditionalTriggersArray.add(conditionalTrigger.name);
        config.put("conditionalTriggers", conditionalTriggers);

        try {
            File file=new File("Vortex/config.json");  
            file.createNewFile();
            FileWriter fileWriter = new FileWriter(file);

            fileWriter.write(config.toJSONString());  
            fileWriter.flush();
            fileWriter.close();  
        } catch (IOException e) { e.printStackTrace(); }  
    }
}