package frc.robot.util.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import frc.robot.util.PIDConstants;

public class NEOSparkMax extends CANSparkMax {
	public NEOSparkMax(int deviceId) {
		super(deviceId, MotorType.kBrushless);
	}

	public NEOSparkMax(int deviceId, PIDConstants constants) {
		super(deviceId, MotorType.kBrushless);
		getPIDController().setP(constants.kP);
		getPIDController().setI(constants.kI);
		getPIDController().setD(constants.kD);
		getPIDController().setFF(constants.kF);
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

	public void configurePID(PIDConstants pid) {
		getPIDController().setP(pid.kP);
		getPIDController().setP(pid.kI);
		getPIDController().setP(pid.kD);
		getPIDController().setFF(pid.kF);
	}
}
