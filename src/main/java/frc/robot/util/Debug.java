package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Debug {
	/**
	 * @param key Label on SmartDashboard
	 * @param val Default value
	 * @return Value on SmartDashboard
	 */
	public static double debugNumber(String key, double val) {
		SmartDashboard.setDefaultNumber(key, val);
		return SmartDashboard.getNumber(key, 0.0);
	}

	/**
	 * @param key Label on SmartDashboard
	 * @param val Default value
	 * @return Value on SmartDashboard
	 */
	public static boolean debugBool(String key, boolean val) {
		SmartDashboard.setDefaultBoolean(key, val);
		return SmartDashboard.getBoolean(key, false);
	}
}
