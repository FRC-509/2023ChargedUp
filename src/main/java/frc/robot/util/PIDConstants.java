package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.interfaces.IDebuggable;

public class PIDConstants implements IDebuggable {
	public double kP;
	public double kI;
	public double kD;
	public double kF;

	// debug fields
	public ControlMode mode;
	private boolean halt;
	private double target;

	public PIDConstants(double kP, double kI, double kD, double kF) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.mode = ControlMode.Velocity;
	}

	public PIDConstants(ControlMode mode, double kP, double kI, double kD, double kF) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.mode = mode;
	}

	public void configureTalonFX(TalonFX talon) {
		talon.config_kP(0, kP);
		talon.config_kI(0, kI);
		talon.config_kD(0, kD);
		talon.config_kF(0, kF);
	}

	@Override
	public void show(String key) {
		SmartDashboard.putNumber(key + " kP", kP);
		SmartDashboard.putNumber(key + " kI", kI);
		SmartDashboard.putNumber(key + " kD", kD);
		SmartDashboard.putNumber(key + " kF", kF);

	}

	@Override
	public void debug(String key) {
		Debug.debugNumber(key + " kP", kP);
		Debug.debugNumber(key + " kI", kI);
		Debug.debugNumber(key + " kD", kD);
		Debug.debugNumber(key + " kF", kF);

		switch (mode) {
			case Velocity:
				Debug.debugNumber(key + " target velocity", target);
				break;
			case Position:
				Debug.debugNumber(key + " target position", target);
				break;
			default:
				SmartDashboard.putString(key + " unsupported control mode", mode.toString());
				break;
		}
	}
}