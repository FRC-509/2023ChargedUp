package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class JoystickController extends Joystick {
	public enum StickButton {
		Trigger(1),
		Bottom(2),
		Left(3),
		Right(4),
		LeftSideLeftTop(5),
		LeftSideMiddleTop(6),
		LeftSideRightTop(7),
		LeftSideRightBottom(8),
		LeftSideMiddleBottom(9),
		LeftSideLeftBottom(10),
		RightSideRightTop(11),
		RightSideMiddleTop(12),
		RightSideLeftTop(13),
		RightSideLeftBottom(14),
		RightSideMiddleBottom(15),
		RightSideRightBottom(16);

		public final int id;

		StickButton(int id) {
			this.id = id;
		}

		public int id() {
			return id;
		}
	}

	public JoystickController(int port) {
		super(port);
	}

	public JoystickButton getJoystickButton(StickButton button) {
		return new JoystickButton(this, button.id());
	}

	public boolean isPressed(StickButton button) {
		return super.getRawButtonPressed(button.id());
	}

	public void isPressedBind(StickButton button, Command command) {
		getJoystickButton(button).toggleOnTrue(command);
	}

	public boolean isReleased(StickButton button) {
		return super.getRawButtonReleased(button.id());
	}

	public void isReleasedBind(StickButton button, Command command) {
		getJoystickButton(button).toggleOnFalse(command);
	}

	public boolean isDown(StickButton button) {
		return super.getRawButton(button.id());
	}

	public void isDownBind(StickButton button, Command command) {
		getJoystickButton(button).whileTrue(command);
	}

	public boolean isUp(StickButton button) {
		return !isDown(button);
	}

	public void isUpBind(StickButton button, Command command) {
		getJoystickButton(button).whileFalse(command);
	}
}