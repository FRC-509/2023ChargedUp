
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Chassis;
import frc.robot.Constants.ArmDimensions;
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
		double initialExtension = (extensionSRXEncoder.getSelectedSensorPosition() / 4096) * 28.06685 + 240;
		extensionMotor.setSensorPosition(initialExtension);
	}

	public double getPivotDegrees() {
		return Conversions.falconToDegrees(leftPivotMotor.getSelectedSensorPosition(), Constants.pivotGearRatio);
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

	/// Derived equation for extension ticks to length using regression
	public double extensionTicksToLength(double x) {
		return 5.03 + 0.25 * x + 1.9 * Math.sin(Math.toRadians(0.0191 * x) - 0.775);
	}

	/// Derived equation for extension length to ticks using regression
	public double extensionLengthToTicks(double y) {
		return -21.454 + 3.995 * y + 7.794 * Math.sin(Math.toRadians(0.0736 * y) + 1.89);
	}

	public double getExtensionLength() {
		return extensionTicksToLength(getExtensionPosition());
	}

	public double maxPossibleHeightAt(double degrees) {
		double base = getArmLength() * Math.sin(Math.toRadians(degrees));

		if (base < Chassis.width / 2.0d) {
			return Chassis.height;
		} else {
			return 0.1d;
		}
	}

	// TODO: FIXME! - measure from top to ground, not from ground!!!
	public double maxPossibleHeight() {
		return maxPossibleHeightAt(getPivotDegrees());
	}

	/// returns whether a given (pivot, extension) is possible within the
	/// dimensional constraints of the robot
	/// NOTE: pivot should be within [0, 180] degrees
	public boolean isPossible(double pivot, double extension) {
		pivot = MathUtil.clamp(pivot, 0, 180);
		double height = getArmLength() * Math.cos(Math.toRadians(pivot));

		return height > maxPossibleHeight();
	}

	/// Returns the maximum possible extension given the
	/// current pivot heading of the arm
	public double getMaximumExtension() {
		double sin = Math.sin(Math.toRadians(getPivotDegrees()));
		double cos = Math.cos(Math.toRadians(getPivotDegrees()));

		double minHeight = getArmLength() * sin <= Chassis.width / 2
				? Chassis.height
				: 0.1;

		return (minHeight / cos) - ArmDimensions.base;
	}

	public void setExtensionLength(double meters) {
		double ticks = extensionLengthToTicks(meters);
		setExtensionPosition(ticks);
	}

	// Pivot Control

	public void moveArmByDegrees(double deltaDegrees) {
		double targetPosition = Conversions.degreesToFalcon(getPivotDegrees() + deltaDegrees, Constants.pivotGearRatio);
		leftPivotMotor.set(ControlMode.Position, targetPosition);
		rightPivotMotor.set(ControlMode.Position, targetPosition);
	}

	public void setPivotOutput(double percentOutput) {
		percentOutput = MathUtil.clamp(percentOutput, -1.0d, 1.0d);

		leftPivotMotor.set(ControlMode.PercentOutput, percentOutput);
		rightPivotMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	// Extension Control

	public double getExtensionPosition() {
		return extensionMotor.getSensorPosition();
	}

	/// Returns the extension of the arm, converting sensor ticks to extended meters
	/// Derived through data collection and regression

	public double getArmLength() {
		return ArmDimensions.base + getExtensionLength();
	}

	public void setExtensionOutput(double percentOutput, boolean disableSoftStop) {
		extensionLoopDisabled = true;

		if (percentOutput > 0 && extensionMotor.getSensorPosition() >= Constants.maxExtension) {
			extensionMotor.set(0);
		} else {
			extensionMotor.set(percentOutput);
		}
	}

	public void setExtensionPosition(double targetPos) {
		extensionLoopDisabled = false;
		targetExtension = MathUtil.clamp(targetPos, 0, Constants.maxExtension);
	}

	public void stopExtensionMotor() {
		setExtensionOutput(0, false);
	}

	public void extensionAtSetpoint() {
		extensionPositionPID.atSetpoint();
	}

	@Override
	public void periodic() {
		if (!extensionLoopDisabled) {
			double output = extensionPositionPID.calculate(extensionMotor.getSensorPosition(), targetExtension);
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
				extensionSRXEncoder.getSelectedSensorPosition() / 4096 * 28.06685 + 240);
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
