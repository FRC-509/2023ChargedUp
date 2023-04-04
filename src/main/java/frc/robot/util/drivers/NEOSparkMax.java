package frc.robot.util.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import frc.robot.util.PIDWrapper;

public class NEOSparkMax extends CANSparkMax {
	public NEOSparkMax(int deviceId) {
		super(deviceId, MotorType.kBrushless);
	}

	public NEOSparkMax(int deviceId, PIDWrapper constants) {
		super(deviceId, MotorType.kBrushless);
		getPIDController().setP(constants.getP());
		getPIDController().setI(constants.getI());
		getPIDController().setD(constants.getD());
		getPIDController().setFF(constants.getF());
	}

	public double getSensorPosition() {
		return getEncoder().getPosition();
	}

	public double getSensorVelocity() {
		return getEncoder().getVelocity();
	}

	public REVLibError setSensorPosition(double position) {
		return getEncoder().setPosition(position);
	}

	public REVLibError setSensorPositionConversionFactor(double factor) {
		return getEncoder().setPositionConversionFactor(factor);
	}

	public REVLibError setSensorVelocityConversionFactor(double factor) {
		return getEncoder().setVelocityConversionFactor(factor);
	}

	public void setReference(double value, CANSparkMax.ControlType controlType) {
		getPIDController().setReference(value, controlType);
	}

	public void configurePID(PIDWrapper pid) {
		getPIDController().setP(pid.getP());
		getPIDController().setP(pid.getI());
		getPIDController().setP(pid.getD());
		getPIDController().setFF(pid.getF());
	}
}
