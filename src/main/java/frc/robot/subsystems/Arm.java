
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PIDConstants;
import frc.robot.util.Conversions;
import frc.robot.util.Device;
import frc.robot.util.drivers.LazyTalonFX;
import frc.robot.util.drivers.NEOSparkMax;

public class Arm extends SubsystemBase {
	private final LazyTalonFX leftPivotMotor;
	private final LazyTalonFX rightPivotMotor;
	private PIDController extensionPID;

	// the percent output needed to maintain a completely horizontal position.
	private static final double maximumFF = 0;
	private final NEOSparkMax extensionMotor;
	private double targetExtension;
	private boolean extensionLoopDisabled;

	/*
	 * Arm MUST start in a downward rotation, and completely retracted with an
	 * extension of 0.
	 */
	public Arm() {
		leftPivotMotor = Device.Motor.leftPivot.build();
		rightPivotMotor = Device.Motor.rightPivot.build();
		extensionMotor = Device.Motor.extension.build();

		targetExtension = 0;
		extensionLoopDisabled = false;
		extensionPID = new PIDController(0.1, 0.0, 0.0);
		extensionPID.setTolerance(7.5);

		extensionMotor.setSmartCurrentLimit(20);
		extensionMotor.setSensorPosition(0);
	}

	public void tunePID() {
		double kP = Conversions.serializeNumber("ExkP", 1.0);
		double kI = Conversions.serializeNumber("ExkI", 0.0);
		double kD = Conversions.serializeNumber("ExkD", 0.0);

		SmartDashboard.putNumber("current rotation: ", getPivotDegrees());

		leftPivotMotor.config_kP(0, kP);
		leftPivotMotor.config_kI(0, kI);
		leftPivotMotor.config_kD(0, kD);

		rightPivotMotor.config_kP(0, kP);
		rightPivotMotor.config_kI(0, kI);
		rightPivotMotor.config_kD(0, kD);

		double target = Conversions.serializeNumber("arm rotation: ", 0.0);

		leftPivotMotor.set(ControlMode.Position, Conversions.degreesToFalcon(target, Constants.pivotGearRatio));
		rightPivotMotor.set(ControlMode.Position, Conversions.degreesToFalcon(target, Constants.pivotGearRatio));
	}

	public double getPivotDegrees() {
		return Conversions.falconToDegrees(leftPivotMotor.getSelectedSensorPosition(), Constants.pivotGearRatio);
	}

	public double getExtensionPosition() {
		return extensionMotor.getSensorPosition();
	}

	public void setPivotOutput(double percentOutput) {
		SmartDashboard.putNumber("pivot percent ou", percentOutput);
		percentOutput = MathUtil.clamp(percentOutput, -1.0d, 1.0d);

		leftPivotMotor.set(ControlMode.PercentOutput, percentOutput);
		rightPivotMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	public void setPivotDegrees(double angle) {
		// calculate "effective" delta angle and add back to raw encoder value to allow
		// for continuous control
		double bounded = getPivotDegrees() % 360.0d;
		double targetPosition = Conversions.degreesToFalcon(getPivotDegrees() + (angle - bounded),
				Constants.pivotGearRatio);
		double feedforward = Math.cos(Math.toRadians(getPivotDegrees())) * maximumFF;
		leftPivotMotor.set(ControlMode.Position, targetPosition, DemandType.ArbitraryFeedForward, feedforward);
		rightPivotMotor.set(ControlMode.Position, targetPosition, DemandType.ArbitraryFeedForward, feedforward);
	}

	public void moveArmBy(double deltaDegrees) {
		// calculate "effective" delta angle and add back to raw encoder value to allow
		// for continuous control
		double targetPosition = Conversions.degreesToFalcon(getPivotDegrees() + deltaDegrees, Constants.pivotGearRatio);
		leftPivotMotor.set(ControlMode.Position, targetPosition);
		rightPivotMotor.set(ControlMode.Position, targetPosition);
	}

	/*
	 * public void resetExtensionSensorPosition() {
	 * extensionMotor.setSensorPosition(0);
	 * }
	 */

	public void setExtensionPosition(double targetPos) {
		extensionLoopDisabled = false;
		targetExtension = MathUtil.clamp(targetPos, 0, Constants.maxExtension);
	}

	public void setExtensionOutput(double percentOutput, boolean disableSoftStop) {
		extensionLoopDisabled = true;
		if (extensionMotor.getSensorPosition() >= 0 && extensionMotor.getSensorPosition() <= Constants.maxExtension
				|| disableSoftStop) {
			extensionMotor.set(percentOutput);
		} else {
			extensionMotor.set(0);
		}
	}

	public void stopExtensionMotor() {
		setExtensionOutput(0, false);
	}

	@Override
	public void periodic() {

		// We put our extension PID loop logic in a periodic() method so that its always
		// running.
		if (!extensionLoopDisabled) {
			double output = extensionPID.calculate(extensionMotor.getSensorPosition(), targetExtension);
			extensionMotor.set(output);
		}

		SmartDashboard.putNumber("Arm Pviot (Degrees)", getPivotDegrees());
		SmartDashboard.putNumber("Arm Extension Current (Amps)", extensionMotor.getOutputCurrent());
		SmartDashboard.putNumber("Arm Extension Position", extensionMotor.getSensorPosition());
	}
}
