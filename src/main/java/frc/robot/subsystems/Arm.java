
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
import frc.robot.util.Utils;
import frc.robot.util.drivers.LazyTalonFX;
import frc.robot.util.drivers.NEOSparkMax;

public class Arm extends SubsystemBase {
	private final LazyTalonFX pivotMotor1 = new LazyTalonFX(20);
	private final LazyTalonFX pivotMotor2 = new LazyTalonFX(13);
	private PIDController extensionPID;
	private static final PIDConstants pivotConstants = new PIDConstants(0.03, 0, 0, 0);

	// the percent output needed to maintain a completely horizontal position.
	private static final double maximumFF = 0;
	private final NEOSparkMax extensionMotor = new NEOSparkMax(12);
	private double targetExtension;
	private boolean extensionLoopDisabled;

	/*
	 * Arm MUST start in a downward rotation, and completely retracted with an
	 * extension of 0.
	 */
	public Arm() {
		targetExtension = 0;
		extensionLoopDisabled = false;
		extensionPID = new PIDController(0.1, 0.0, 0.0);
		extensionPID.setTolerance(7.5);

		extensionMotor.setSmartCurrentLimit(20);
		extensionMotor.setIdleMode(IdleMode.kBrake);
		extensionMotor.setSensorPosition(0);
		// Zero pivot encoders
		pivotMotor1.setSelectedSensorPosition(0);
		pivotMotor2.setSelectedSensorPosition(0);
		pivotMotor1.setInverted(false);
		pivotMotor2.setInverted(true);
		pivotMotor1.setNeutralMode(NeutralMode.Brake);
		pivotMotor2.setNeutralMode(NeutralMode.Brake);
		// Configure PID constants
		pivotConstants.configureTalonFX(pivotMotor1);
		pivotConstants.configureTalonFX(pivotMotor2);
	}

	public void tunePID() {
		double kP = Utils.serializeNumber("ExkP", 1.0);
		double kI = Utils.serializeNumber("ExkI", 0.0);
		double kD = Utils.serializeNumber("ExkD", 0.0);

		SmartDashboard.putNumber("current rotation: ", getPivotDegrees());

		pivotMotor1.config_kP(0, kP);
		pivotMotor1.config_kI(0, kI);
		pivotMotor1.config_kD(0, kD);

		pivotMotor2.config_kP(0, kP);
		pivotMotor2.config_kI(0, kI);
		pivotMotor2.config_kD(0, kD);

		double target = Utils.serializeNumber("arm rotation: ", 0.0);

		pivotMotor1.set(ControlMode.Position, Utils.degreesToFalcon(target, Constants.pivotGearRatio));
		pivotMotor2.set(ControlMode.Position, Utils.degreesToFalcon(target, Constants.pivotGearRatio));
	}

	public double getPivotDegrees() {
		return Utils.falconToDegrees(pivotMotor1.getSelectedSensorPosition(), Constants.pivotGearRatio);
	}

	public double getExtensionPosition() {
		return extensionMotor.getSensorPosition();
	}

	public void setPivotOutput(double percentOutput) {
		SmartDashboard.putNumber("pivot percent ou", percentOutput);
		percentOutput = MathUtil.clamp(percentOutput, -1.0d, 1.0d);

		pivotMotor1.set(ControlMode.PercentOutput, percentOutput);
		pivotMotor2.set(ControlMode.PercentOutput, percentOutput);
	}

	public void setPivotDegrees(double angle) {
		// calculate "effective" delta angle and add back to raw encoder value to allow
		// for continuous control
		double bounded = getPivotDegrees() % 360.0d;
		double targetPosition = Utils.degreesToFalcon(getPivotDegrees() + (angle - bounded), Constants.pivotGearRatio);
		double feedforward = Math.cos(Math.toRadians(getPivotDegrees())) * maximumFF;
		pivotMotor1.set(ControlMode.Position, targetPosition, DemandType.ArbitraryFeedForward, feedforward);
		pivotMotor2.set(ControlMode.Position, targetPosition, DemandType.ArbitraryFeedForward, feedforward);
	}

	public void moveArmBy(double deltaDegrees) {
		// calculate "effective" delta angle and add back to raw encoder value to allow
		// for continuous control
		double targetPosition = Utils.degreesToFalcon(getPivotDegrees() + deltaDegrees, Constants.pivotGearRatio);
		pivotMotor1.set(ControlMode.Position, targetPosition);
		pivotMotor2.set(ControlMode.Position, targetPosition);
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
