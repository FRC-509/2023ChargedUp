
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LazyTalonFX;
import frc.robot.util.Utils;

public class Arm extends SubsystemBase {
  private final LazyTalonFX pivotMotor1 = new LazyTalonFX(20);
  private final LazyTalonFX pivotMotor2 = new LazyTalonFX(13);

  private final CANSparkMax extensionMotor = new CANSparkMax(12, MotorType.kBrushless);
  private SparkMaxPIDController extensionController;
  private RelativeEncoder extensionEncoder;

  private static final double pivotGearRatio = 256.0d;
  // private static final double extensionGearRatio = 256.0d;

  // Dummy!
  private static final double maxExtension = 100;
  private static final Utils.PIDConstants pivotConstants = new Utils.PIDConstants(1.0, 0, 0, 0);
  private static final Utils.PIDConstants extensionConstants = new Utils.PIDConstants(1.0, 0, 0, 0);

  /*
   * Arm MUST start in a downward rotation, and completely retracted with an
   * extension of 0.
   */
  public Arm() {
    extensionMotor.setSmartCurrentLimit(15);
    extensionMotor.setIdleMode(IdleMode.kBrake);
    // Zero pivot encoders
    pivotMotor1.setSelectedSensorPosition(0);
    pivotMotor2.setSelectedSensorPosition(0);
    pivotMotor1.setInverted(true);
    pivotMotor2.setInverted(false);
    pivotMotor1.setNeutralMode(NeutralMode.Brake);
    pivotMotor2.setNeutralMode(NeutralMode.Brake);
    pivotMotor1.setSelectedSensorPosition(0);
    pivotMotor2.setSelectedSensorPosition(0);
  }

  public void setPivotOutput(double output) {
    SmartDashboard.putNumber("Arm output current", extensionMotor.getOutputCurrent());

    SmartDashboard.putNumber("Arm output current", extensionMotor.getEncoder().getPosition());
    output = MathUtil.clamp(output, -1.0d, 1.0d);
    pivotMotor1.set(ControlMode.PercentOutput, output);
    pivotMotor2.set(ControlMode.PercentOutput, output);
}
  public double getArmDegrees() {
    return Utils.falconToDegrees(pivotMotor1.getSelectedSensorPosition(), pivotGearRatio);
  }

  public boolean inDanger() {
    return getArmDegrees() >= -50 && getArmDegrees() <= -5;
  }

  public void setExtensionOutput(double output) {
      output = MathUtil.clamp(output, -1.0d, 1.0d);
      extensionMotor.set(output);
  }
  @Override
  public void periodic() {
  }
}
