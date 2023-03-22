package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.interfaces.IDebuggable;

public class PIDWrapper extends PIDController implements IDebuggable {
	private double kF;

	public double getF() {
		return kF;
	}

	public void setF(double kF) {
		this.kF = kF;
	}

	public PIDWrapper() {
		super(0.0d, 0.0d, 0.0d);
		setF(0.0d);
	}

	public PIDWrapper(double kP, double kI, double kD, double kF) {
		super(kP, kI, kD);
		setF(kF);
	}

	public PIDWrapper(PIDController wrapper) {
		super(wrapper.getP(), wrapper.getI(), wrapper.getD());
		setF(0.0d);
	}

	public PIDWrapper(PIDWrapper wrapper) {
		super(wrapper.getP(), wrapper.getI(), wrapper.getD());
		setF(wrapper.getF());
	}

	public void configureTalonFX(TalonFX talon) {
		talon.config_kP(0, getP());
		talon.config_kI(0, getI());
		talon.config_kD(0, getD());
		talon.config_kF(0, kF);
	}

	@Override
	public void debug(String key) {
		double kP = Debug.debugNumber(key + " kP", getP());
		double kI = Debug.debugNumber(key + " kI", getI());
		double kD = Debug.debugNumber(key + " kD", getD());
		double kF = Debug.debugNumber(key + " kF", getF());

		setP(kP);
		setI(kI);
		setD(kD);
		setF(kF);
	}

	@Override
	public void show(String key) {
		// TODO Auto-generated method stub

	}
}