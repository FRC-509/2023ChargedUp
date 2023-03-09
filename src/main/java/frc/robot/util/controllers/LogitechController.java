package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class LogitechController extends GenericHID {
	public enum LogiButton {
		A(2),
		B(3),
		X(1),
		Y(4),
		LBTrigger(5),
		RBTrigger(6),
		LTrigger(7),
		RTrigger(8),
		Back(9),
		Start(10),
		LStick(11),
		RStick(12);

		public final int id;

		LogiButton(int id) {
			this.id = id;
		}

		public int id() {
			return id;
		}
	}

	public enum LogiAxis {
		LeftStickX(0),
		LeftStickY(1),
		RightStickX(2),
		RightStickY(3);

		public final int id;

		LogiAxis(int id) {
			this.id = id;
		}

		public int id() {
			return id;
		}
	}

	public LogitechController(int port) {
		super(port);
	}

	public JoystickButton getControllerButton(LogiButton button) {
		return new JoystickButton(this, button.id());
	}

	public boolean isPressed(LogiButton button) {
		return super.getRawButtonPressed(button.id());
	}

	public void isPressedBind(LogiButton button, Command command) {
		getControllerButton(button).toggleOnTrue(command);
	}

	public boolean isReleased(LogiButton button) {
		return super.getRawButtonReleased(button.id());
	}

	public void isReleasedBind(LogiButton button, Command command) {
		getControllerButton(button).toggleOnFalse(command);
	}

	public boolean isDown(LogiButton button) {
		return super.getRawButton(button.id());
	}

	public void isDownBind(LogiButton button, Command command) {
		getControllerButton(button).whileTrue(command);
	}

	public boolean isUp(LogiButton button) {
		return !isDown(button);
	}

	public void isUpBind(LogiButton button, Command command) {
		getControllerButton(button).whileFalse(command);
	}

	public double getLeftStickX() {
		return super.getRawAxis(LogiAxis.LeftStickX.id());
	}

	public double getLeftStickY() {
		return super.getRawAxis(LogiAxis.LeftStickY.id());
	}

	public double getRightStickX() {
		return super.getRawAxis(LogiAxis.RightStickX.id());
	}

	public double getRightStickY() {
		return super.getRawAxis(LogiAxis.RightStickY.id());
	}
	/*
	 * public double getTriggers() {
	 * return super.getRawAxis(LogiAxis.Triggers.id());
	 * }
	 */
}