
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Debug;
import frc.robot.util.Device;
import frc.robot.util.drivers.LazyTalonFX;
import frc.robot.util.drivers.NEOSparkMax;
import frc.robot.util.interfaces.IDebuggable;
import frc.robot.util.math.Conversions;
import frc.robot.util.math.PositionTarget;

public class Arm extends SubsystemBase implements IDebuggable {
	private final LazyTalonFX leftPivotMotor;
	private final LazyTalonFX rightPivotMotor;
	private CANCoder pivotEncoder;
	private PIDController extensionPositionPID;
	private TalonSRX extensionSRXEncoder;
	private PositionTarget extensionTarget;
	private PositionTarget pivotTarget;

	// the percent output needed to maintain a completely horizontal position.
	private final NEOSparkMax extensionMotor;
	private double targetExtension;
	private boolean extensionLoopDisabled;
	private double pivotFeedForward = 0.0;

	public Arm() {
		this.leftPivotMotor = Device.Motor.leftPivot.build();
		this.rightPivotMotor = Device.Motor.rightPivot.build();
		this.extensionMotor = Device.Motor.extension.build();
		this.pivotEncoder = Device.Encoder.pivot.build();

		extensionMotor.setSmartCurrentLimit(20);
		extensionMotor.setSensorPosition(0);
		extensionSRXEncoder = new TalonSRX(Device.EncoderId.extension);
		extensionPositionPID = Constants.PID.extension_P;

		targetExtension = 0;
		extensionLoopDisabled = true;

		Timer.delay(1.0);

		double initialPivot = Conversions.degreesToFalcon(pivotEncoder.getAbsolutePosition(), Constants.pivotGearRatio);
		leftPivotMotor.setSelectedSensorPosition(initialPivot);
		rightPivotMotor.setSelectedSensorPosition(initialPivot);

		extensionSRXEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		double initialExtension = (extensionSRXEncoder.getSelectedSensorPosition() / 4096) * 28.06685 - 23.5;
		extensionMotor.setSensorPosition(initialExtension);

		this.extensionTarget = new PositionTarget(
				getExtensionPosition(),
				0.0d,
				Constants.Arm.maxExtension);
		this.pivotTarget = new PositionTarget(
				getPivotDegrees(),
				0,
				150);
	}

	/**
	 * @return the current extension of the arm in sensor ticks
	 */
	public double getExtensionPosition() {
		return extensionMotor.getSensorPosition();
	}

	/**
	 * @return the current pivot of the arm in degrees
	 */
	public double getPivotDegrees() {
		return Conversions.falconToDegrees(leftPivotMotor.getSelectedSensorPosition(), Constants.pivotGearRatio);
	}

	/**
	 * @return the current length of the arm extension
	 */
	public double getExtensionLength() {
		return extensionTicksToLength(getExtensionPosition());
	}

	/**
	 * @return the current length of the arm base + extension
	 */
	public double getArmLength() {
		return Constants.Arm.baseLength + getExtensionLength();
	}

	/**
	 * @return the maximum possible length of the arm base + extension
	 */
	public double getMaxArmLength() {
		return Constants.Arm.maxExtensionLength + Constants.Arm.baseLength;
	}

	/**
	 * @return the maximum possible length of the arm base + extension
	 */
	public double getMinArmLength() {
		return Constants.Arm.minExtensionLength + Constants.Arm.baseLength;
	}

	/**
	 * @param degrees the pivot of the arm in degrees
	 * @return the maximum arm length allowed given the pivot degrees
	 *         <p>
	 *         If the arm can intersect the chassis base at the given rotation
	 *         the maximum height relates to the chassis height, otherwise it
	 *         relates to the minimum allowed height of the arm
	 */
	public double getHeightLimitAt(double degrees) {
		if (degrees > 90.0d && degrees < 270.0d) {
			return Double.NEGATIVE_INFINITY;
		}

		if (isInChassis()) {
			return Constants.Arm.pivotHeight - Constants.Chassis.height;
		} else {
			return Constants.Arm.pivotHeight - Constants.Arm.minHeight;
		}
	}

	/**
	 * @return the current maximum arm length allowed
	 */
	public double getHeightLimit() {
		return getHeightLimitAt(getPivotDegrees());
	}

	/**
	 * @return the height of the arm, or l * sin(theta)
	 */
	public double getHeight() {
		return getArmLength() * Math.cos(Math.toRadians(getPivotDegrees()));
	}

	/**
	 * @param pivot     an arm pivot in degrees
	 * @param extension an extension length in meters
	 * @return whether the provided state is allowed given the dimensions of the
	 *         robot
	 */
	public boolean isValidState(double pivot, double extension) {
		double height = getArmLength() * Math.cos(Math.toRadians(pivot));
		return height < getHeightLimit();
	}

	/**
	 * @return whether the arm is currently residing within the chassis bounds
	 */
	public boolean isInChassis() {
		double sin = getArmLength() * Math.sin(Math.toRadians(getPivotDegrees()));
		return sin < Constants.Arm.offsetToBase;
	}

	/**
	 * @param x the extension motor ticks
	 * @return the ticks converted into extension length
	 *         <p>
	 *         The equation for extension ticks to length was derived using
	 *         regression
	 */
	public double extensionTicksToLength(double x) {
		return 5.03 + 0.25 * x + 1.9 * Math.sin(Math.toRadians(0.0191 * x) - 0.775);
	}

	/**
	 * @param y the extension length in meters
	 * @return the length converted into extension ticks
	 *         <p>
	 *         The equation for extension length to ticks was derived using
	 *         regression
	 */
	public double extensionLengthToTicks(double y) {
		return -21.454 + 3.995 * y + 7.794 * Math.sin(Math.toRadians(0.0736 * y) + 1.89);
	}

	public void setPivotDegrees(double degrees) {
		double delta = (degrees - getPivotDegrees()) % 360;
		if (delta > 180.0d) {
			delta -= 360.0d;
		} else if (delta < -180.0d) {
			delta += 360.0d;
		}

		double target = getPivotDegrees() + delta;
		double targetDegrees = Conversions.degreesToFalcon(target, Constants.pivotGearRatio);
		leftPivotMotor.set(ControlMode.Position, targetDegrees);
		rightPivotMotor.set(ControlMode.Position, targetDegrees);
	}

	public void setExtensionLength(double meters) {
		double ticks = extensionLengthToTicks(meters);
		setExtensionPosition(ticks);
	}

	public void setPivotOutput(double percent) {
		double target = pivotTarget.update(percent, Constants.Arm.maxPivotSpeed);

		leftPivotMotor.set(ControlMode.Position, target);
		rightPivotMotor.set(ControlMode.Position, target);
	}

	public void setExtensionOutput(double percent) {
		// TODO: meters / second or units / second?
		extensionTarget.update(percent, Constants.Arm.maxExtensionSpeed);
	}

	// Extension Control

	public void setExtensionPosition(double target) {
		extensionLoopDisabled = false;
		extensionTarget.setPosition(target);
	}

	public void stopExtensionMotor() {
		setExtensionOutput(0);
	}

	@Override
	public void periodic() {
		if (!extensionLoopDisabled) {
			double output = extensionPositionPID.calculate(extensionMotor.getSensorPosition(),
					extensionTarget.getTarget());
			extensionMotor.set(output);
		}

		show("s_arm");
		// debug("s_arm")
	}

	@Override
	public void show(String key) {
		SmartDashboard.putNumber("Arm Pivot (Degrees)", getPivotDegrees());
		SmartDashboard.putNumber("Arm (Int) Extension Position", extensionMotor.getSensorPosition());
		SmartDashboard.putNumber("Arm Extension (dummy talon srx)",
				extensionSRXEncoder.getSelectedSensorPosition() / 4096 * 28.06685 - 23.5);
		SmartDashboard.putNumber("Arm Pivot CANCODER", pivotEncoder.getAbsolutePosition());
	}

	@Override
	public void debug(String key) {
		leftPivotMotor.getConstants().debug("pivot");
		rightPivotMotor.setConstants(leftPivotMotor.getConstants());

		SmartDashboard.putNumber("current rotation: ", getPivotDegrees());
		double target = Debug.debugNumber("arm target velocity: ", 0.0);

		leftPivotMotor.set(ControlMode.Velocity, Conversions.degreesToFalcon(target, Constants.pivotGearRatio));
		rightPivotMotor.set(ControlMode.Velocity, Conversions.degreesToFalcon(target, Constants.pivotGearRatio));
	}
}
