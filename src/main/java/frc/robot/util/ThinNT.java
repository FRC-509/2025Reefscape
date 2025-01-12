package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;

public class ThinNT {
	public static void putNumber(String key, double value) {
		NetworkTableInstance.getDefault().getTable("Translucent").getEntry(key).setValue(value);
	}

	public static void putBoolean(String key, boolean value) {
		NetworkTableInstance.getDefault().getTable("Translucent").getEntry(key).setValue(value);
	}

	public static void putString(String key, String value) {
		NetworkTableInstance.getDefault().getTable("Translucent").getEntry(key).setValue(value);
	}
}
