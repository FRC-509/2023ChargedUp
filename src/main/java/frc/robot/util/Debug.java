package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Debug {

	private static ArrayList<String> serializedIds = new ArrayList<String>();

	/**
	 * @param key Label on SmartDashboard
	 * @param val Default value
	 * @return Value on SmartDashboard
	 */
	public static double serializeNumber(String key, double val) {
		serializedIds.add(key);
		SmartDashboard.setDefaultNumber(key, val);
		return SmartDashboard.getNumber(key, 0.0);
	}

	/**
	 * @param key Label on SmartDashboard
	 * @param val Default value
	 * @return Value on SmartDashboard
	 */
	public static boolean serializeBoolean(String key, boolean val) {
		serializedIds.add(key);
		SmartDashboard.setDefaultBoolean(key, val);
		return SmartDashboard.getBoolean(key, false);
	}

	public static void flushShuffleboard() {
		for (String key : serializedIds) {
			SmartDashboard.clearPersistent(key);
		}
	}
}
