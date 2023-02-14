
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LazyTalonFX;
import frc.robot.Utils;
import frc.robot.Utils.PIDConstants;

public class Arm extends SubsystemBase {
  private final LazyTalonFX pivotMotor1 = new LazyTalonFX(12);
  private final LazyTalonFX pivotMotor2 = new LazyTalonFX(13);

  private final CANSparkMax extensionMotor = new CANSparkMax(14, MotorType.kBrushless);
  private SparkMaxPIDController extensionController;
  private RelativeEncoder extensionEncoder;

  private static final double pivotGearRatio = 20.0d;
  // private static final double extensionGearRatio = 256.0d;

  // Dummy!
  private static final double maxExtension = 100;
  private static final PIDConstants pivotConstants = new PIDConstants(1.0, 0, 0, 0);
  private static final PIDConstants extensionConstants = new PIDConstants(1.0, 0, 0, 0);

  /*
   * Arm MUST start in a downward rotation, and completely retracted with an
   * extension of 0.
   */
  public Arm() {
    pivotConstants.configureTalonFX(pivotMotor1);
    pivotConstants.configureTalonFX(pivotMotor2);
    // Zero the pivot encoders.
    pivotMotor1.setSelectedSensorPosition(0);
    pivotMotor2.setSelectedSensorPosition(0);
    pivotMotor1.setNeutralMode(NeutralMode.Brake);
    pivotMotor2.setNeutralMode(NeutralMode.Brake);
    pivotMotor2.follow(pivotMotor1, FollowerType.AuxOutput1);

    extensionConstants.configureCANSparkMax(extensionMotor);
    extensionController = extensionMotor.getPIDController();
    extensionEncoder = extensionMotor.getEncoder();

    // Zero the extension encoder.
    extensionEncoder.setPosition(0);
  }

  public void setPivotAngle(double targetAngle) {
    double currentAngle = getPivotAngle();
    double delta = targetAngle - (currentAngle % 360.0d);
    double outputAngle = Utils.degreesToFalcon(currentAngle + delta, pivotGearRatio);

    // double lastAngle2 =
    // Utils.falconToDegrees(pivotMotor2.getSelectedSensorPosition(),
    // pivotGearRatio);
    // double delta2 = angle - (lastAngle2 % 360.0d);
    // double outputAngle2 = Utils.degreesToFalcon(lastAngle2 + delta2,
    // pivotGearRatio);

    pivotMotor1.set(ControlMode.Position, outputAngle);
  }

  public void setPercentExtension(double percentExtension) {
    percentExtension = MathUtil.clamp(percentExtension, -1.0, 1.0);
    extensionController.setReference(percentExtension * maxExtension, ControlType.kPosition);
  }

  public double getPivotAngle() {
    return Utils.falconToDegrees(pivotMotor1.getSelectedSensorPosition(), pivotGearRatio);
  }

  public double getPercentExtension() {
    return extensionEncoder.getPosition() / maxExtension;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Extension", extensionEncoder.getPosition());
    SmartDashboard.putNumber("Arm Pivot Angle", getPivotAngle());
  }
}
