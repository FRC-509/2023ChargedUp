
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Device;
import frc.robot.util.PIDWrapper;
import frc.robot.util.drivers.LazyTalonFX;
import frc.robot.util.drivers.NEOSparkMax;
import frc.robot.util.interfaces.IDebuggable;
import frc.robot.util.math.Conversions;
import frc.robot.util.math.PositionTarget;
import frc.robot.util.math.Utils;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class Arm extends SubsystemBase implements IDebuggable {
	private final LazyTalonFX leftPivotMotor;
	private final LazyTalonFX rightPivotMotor;
	private CANCoder pivotEncoder;
	private PIDWrapper extensionPositionPID;
	private PositionTarget extensionTarget;
	private PositionTarget pivotTarget;
	private final NEOSparkMax extensionMotor;
	private boolean zeroed = false;
	private Timer isStationaryTimer;

	private DigitalInput extensionLimit;

	public Arm() {
		this.leftPivotMotor = Device.Motor.leftPivot.build();
		this.rightPivotMotor = Device.Motor.rightPivot.build();
		this.extensionMotor = Device.Motor.extension.build();

		this.pivotEncoder = Device.Encoder.pivot.build();
		extensionMotor.setSmartCurrentLimit(20);
		extensionMotor.setSensorPosition(0);
		extensionPositionPID = new PIDWrapper(Constants.PID.extension_P);
		extensionLimit = new DigitalInput(9);
		isStationaryTimer = new Timer();
		isStationaryTimer.start();
	}

	public void onFirstInit() {
		double pivot = Conversions.degreesToFalcon(pivotEncoder.getAbsolutePosition(),
				Constants.pivotGearRatio);
		leftPivotMotor.setSelectedSensorPosition(pivot);
		rightPivotMotor.setSelectedSensorPosition(pivot);

		SmartDashboard.putNumber("init pivot: ", pivotEncoder.getAbsolutePosition());

		this.extensionTarget = new PositionTarget(
				getExtensionPosition(),
				0.0d,
				Constants.Arm.maxExtension);

		this.pivotTarget = new PositionTarget(
				pivotEncoder.getAbsolutePosition(),
				Constants.Arm.minPivot,
				Constants.Arm.maxPivot);

		resetExtensionPosition();
	}

	public void setState(TrapezoidProfile.State state) {
		setPivotOutput(state.velocity / Constants.Arm.maxPivotSpeed);
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

	public double getPivotVelocityDegreesPerSecond() {
		return Conversions.falconToRPM(leftPivotMotor.getSelectedSensorVelocity(), Constants.pivotGearRatio) * 360.0
				/ 60;
	}

	/**
	 * @return the current length of the arm extension
	 */
	public double getExtensionLength() {
		return extensionTicksToLength(getExtensionPosition());
	}

	public double getPivotCosBiasAt(double degrees) {
		return Constants.Arm.offsetToPivot * Math.cos(Math.toRadians(degrees));
	}

	public double getPivotCosBias() {
		return getPivotCosBiasAt(getPivotDegrees());
	}

	public double getPivotSinBiasAt(double degrees) {
		return Constants.Arm.offsetToPivot * Math.sin(Math.toRadians(degrees));
	}

	public double getPivotSinBias() {
		return getPivotSinBiasAt(getPivotDegrees());
	}

	public double getBiasedArmHeight() {
		double theta1 = Math.acos(Constants.Arm.offsetToPivot / getArmLength());
		double theta2 = Math.toRadians(getPivotDegrees() + 90) - theta1;
		double biasedLength = Math.hypot(Constants.Arm.offsetToPivot, getArmLength());

		return biasedLength * Math.cos(theta2);
	}

	public double getBiasedHeightFromGround() {
		return getHeightLimit() - getBiasedArmHeight();
	}

	public double getBiasedArmBase() {
		double theta1 = Math.acos(Constants.Arm.offsetToPivot / getArmLength());
		double theta2 = Math.toRadians(getPivotDegrees() + 90) - theta1;
		double biasedLength = Math.hypot(Constants.Arm.offsetToPivot, getArmLength());

		return biasedLength * Math.sin(theta2);
	}

	/**
	 * @return the current length of the arm base + extension
	 */
	public double getArmLength() {
		return Constants.Arm.baseLength + Constants.Arm.clawLength + getExtensionLength();
	}

	/**
	 * @return the maximum possible length of the arm base + extension
	 */
	public double getMaxArmLength() {
		return Constants.Arm.maxExtensionLength + Constants.Arm.baseLength + Constants.Arm.clawLength;
	}

	/**
	 * @return the maximum possible length of the arm base + extension
	 */
	public double getMinArmLength() {
		return Constants.Arm.baseLength;
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
			return Double.POSITIVE_INFINITY;
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

	public double getHeightFromGround() {
		return getHeightLimit() - getHeight();
	}

	/**
	 * @param pivot     an arm pivot in degrees
	 * @param extension an extension length in meters
	 * @return whether the provided state is allowed given the dimensions of the
	 *         robot
	 */
	public boolean isValidState(double pivot, double extension) {
		double height = extension * Math.cos(Math.toRadians(pivot));
		return height < getHeightLimitAt(pivot);
	}

	/**
	 * @return whether the arm is currently residing within the chassis bounds
	 */
	public boolean isInChassis() {
		double tan = Constants.Arm.pivotHeight * Math.tan(Math.toRadians(getPivotDegrees()));
		return tan < Constants.Arm.offsetToBase;
	}

	/**
	 * @param x the extension motor ticks
	 * @return the ticks converted into extension length
	 *         <p>
	 *         The equation for extension ticks to length was derived using
	 *         regression
	 */
	public static double extensionTicksToLength(double x) {
		return 0.287 * x;
	}

	/**
	 * @param y the extension length in meters
	 * @return the length converted into extension ticks
	 *         <p>
	 *         The equation for extension length to ticks was derived using
	 *         regression
	 */
	public static double extensionLengthToTicks(double y) {
		return y / 0.287;
	}

	public double NEOToSRXUnits(double units) {
		return units * 0.0364971 + 0.898447;
	}

	public double SRXToNEOUnits(double units) {
		return (units - 0.898447) / 0.0364971;
	}

	public void resetExtensionPosition() {
		extensionMotor.setSensorPosition(0.0d);
		extensionTarget.setTarget(0.0d);
	}

	public void setPivotDegrees(double degrees) {
		if (!isValidState(degrees, getArmLength())) {
			return;
		}

		double delta = (degrees - getPivotDegrees()) % 360;
		if (delta > 180.0d) {
			delta -= 360.0d;
		} else if (delta < -180.0d) {
			delta += 360.0d;
		}

		double target = getPivotDegrees() + delta;
		double ticks = Conversions.degreesToFalcon(target, Constants.pivotGearRatio);

		leftPivotMotor.set(ControlMode.Position, ticks);
		rightPivotMotor.set(ControlMode.Position, ticks);

		pivotTarget.setTarget(target);
	}

	public void setPivotOutput(double percent) {
		double previous = pivotTarget.getTarget();
		percent = MathUtil.clamp(percent, -1.0d, +1.0d);
		pivotTarget.update(percent, Constants.Arm.maxPivotSpeed);

		if (percent < 0.0d && !isValidState(pivotTarget.getTarget(), getArmLength())) {
			pivotTarget.setTarget(previous);
		}

		double delta = (pivotTarget.getTarget() - getPivotDegrees()) % 360;
		if (delta > 180.0d) {
			delta -= 360.0d;
		} else if (delta < -180.0d) {
			delta += 360.0d;
		}

		double target = getPivotDegrees() + delta;
		double ticks = Conversions.degreesToFalcon(target, Constants.pivotGearRatio);

		leftPivotMotor.set(ControlMode.Position, ticks);
		rightPivotMotor.set(ControlMode.Position, ticks);

		setPivotDegrees(pivotTarget.getTarget());
	}

	public void setPivotRawOutput(double percent) {
		leftPivotMotor.set(ControlMode.PercentOutput, percent);
		rightPivotMotor.set(ControlMode.PercentOutput, percent);

		double degrees = getPivotDegrees() % 360.0d;

		if (degrees > 180.0d) {
			degrees -= 360.0d;
		} else if (degrees < -180.0d) {
			degrees += 360.0d;
		}

		pivotTarget.setTarget(degrees);
	}

	public void setExtensionPosition(double target) {
		extensionTarget.setTarget(target);
		double output = extensionPositionPID.calculate(extensionMotor.getSensorPosition(), extensionTarget.getTarget());
		extensionMotor.set(output);
	}

	public void setExtensionLength(double meters) {
		double ticks = extensionLengthToTicks(meters);
		setExtensionPosition(ticks);
	}

	public void setExtensionOutput(double percent) {
		double previous = extensionTarget.getTarget();
		percent = MathUtil.clamp(percent, -1.0d, +1.0d);
		extensionTarget.update(percent, Constants.Arm.maxExtensionSpeed);

		if (percent > 0.0d && !isValidState(pivotTarget.getTarget(), getArmLength())) {
			extensionTarget.setTarget(previous);
		}

		double output = extensionPositionPID.calculate(extensionMotor.getSensorPosition(), extensionTarget.getTarget());
		extensionMotor.set(output);
	}

	public void setExtensionRawOutput(double percent) {
		percent = MathUtil.clamp(percent, -1.0d, +1.0d);
		extensionMotor.set(percent);
	}

	// Extension Control

	public void stopExtensionMotor() {
		setExtensionOutput(0);
	}

	public void setPivotToEncoderValue() {
		double pivot = Conversions.degreesToFalcon(pivotEncoder.getAbsolutePosition(),
				Constants.pivotGearRatio);
		leftPivotMotor.setSelectedSensorPosition(pivot);
		rightPivotMotor.setSelectedSensorPosition(pivot);
	}

	@Override
	public void periodic() {

		if (!extensionLimit.get()) {
			if (!zeroed) {
				resetExtensionPosition();
			}
			zeroed = true;
		} else {
			zeroed = false;
		}

		if (Utils.withinDeadband(pivotEncoder.getVelocity(), 0.0d, 0.01)) {
			if (isStationaryTimer.get() > 0.2d) {
				setPivotToEncoderValue();
				isStationaryTimer.reset();
			}
		} else {
			isStationaryTimer.reset();
		}

		SmartDashboard.putNumber("timer: ", isStationaryTimer.get());

		debug("s_arm");
	}

	@Override
	public void debug(String key) {
		SmartDashboard.putNumber("Arm Pivot (Degrees)", getPivotDegrees());
		SmartDashboard.putNumber("Desired Arm Pivot (Degrees)", pivotTarget.getTarget());
		SmartDashboard.putNumber("Arm Extension", extensionMotor.getSensorPosition());
		SmartDashboard.putNumber("Desired Arm Extension", extensionTarget.getTarget());

		SmartDashboard.putNumber("Height",
				getHeightFromGround());

		SmartDashboard.putBoolean("DesiredIsVallid",
				isValidState(pivotTarget.getTarget(), extensionTarget.getTarget()));
		SmartDashboard.putNumber("expected length: ", getExtensionLength());

		SmartDashboard.putNumber("pivot enoder error: `", pivotEncoder.getAbsolutePosition() - getPivotDegrees());
		SmartDashboard.putNumber("sensor velocity: `", leftPivotMotor.getSelectedSensorVelocity());
	}
}
