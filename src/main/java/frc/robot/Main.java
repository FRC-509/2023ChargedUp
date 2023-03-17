package frc.robot;

import java.util.ArrayList;
import java.util.concurrent.Callable;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what
 * you are doing, do not modify this file except to change the parameter class
 * to the startRobot
 * call.
 */
public final class Main {
	private Main() {

	}

	/**
	 * Main initialization function. Do not perform any initialization here.
	 *
	 * <p>
	 * If you change your main robot class, change the parameter type.
	 */
	public static void main(String... args) {
		new ArmTest(ArmState.Autonomous);

		RobotBase.startRobot(Robot::new);
	}
}
