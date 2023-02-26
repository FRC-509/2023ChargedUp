
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
import frc.robot.Constants;
import frc.robot.util.LazyTalonFX;
import frc.robot.util.NEOSparkMax;
import frc.robot.util.Utils;

public class Arm extends SubsystemBase {
  private final LazyTalonFX pivotMotor1 = new LazyTalonFX(20);
  private final LazyTalonFX pivotMotor2 = new LazyTalonFX(13);

  private static final Utils.PIDConstants pivotConstants = new Utils.PIDConstants(0.2, 0, 0, 0);
  
  private final NEOSparkMax extensionMotor = new NEOSparkMax(12);
  
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
    pivotMotor1.setInverted(false);
    pivotMotor2.setInverted(true);
    pivotMotor1.setNeutralMode(NeutralMode.Brake);
    pivotMotor2.setNeutralMode(NeutralMode.Brake);
    // Configure PID constants
    pivotConstants.configureTalonFX(pivotMotor1);
    pivotConstants.configureTalonFX(pivotMotor2);
  }

  /* 
  double angle = 0;
  public void tunePID() {
    double kP = Utils.serializeNumber("kP", 0.0);
    double kI = Utils.serializeNumber("kI", 0.0);
    double kD = Utils.serializeNumber("kD", 0.0);

    pivotMotor1.config_kP(0, kP);
    pivotMotor2.config_kP(0, kP);
    pivotMotor1.config_kI(0, kI);
    pivotMotor2.config_kI(0, kI);
    pivotMotor1.config_kD(0, kD);
    pivotMotor2.config_kD(0, kD);

    double encoder = Utils.falconToDegrees(pivotMotor1.getSelectedSensorPosition(), pivotGearRatio);

    SmartDashboard.putNumber("arm encoder angle", encoder);

    angle = Utils.serializeNumber("arm angle", angle);
    double falcon = Utils.degreesToFalcon(angle, pivotGearRatio);

    pivotMotor1.set(ControlMode.Position, falcon);
    pivotMotor2.set(ControlMode.Position, falcon);
  }
  */

  public void setPivotOutput(double output) {

    output = MathUtil.clamp(output, -1.0d, 1.0d);
    pivotMotor1.set(ControlMode.PercentOutput, output);
    pivotMotor2.set(ControlMode.PercentOutput, output);
  }

  public void setPivotDegrees(double angle) {
    // calculate "effective" delta angle and add back to raw encoder value to allow for continuous control
    double bounded = getPivotDegrees() % 360.0d;
    double targetPosition = Utils.degreesToFalcon(getPivotDegrees() + (angle - bounded), Constants.pivotGearRatio);

    pivotMotor1.set(ControlMode.Position, targetPosition);
    pivotMotor2.set(ControlMode.Position, targetPosition);
  }

  public double getPivotDegrees() {
    return Utils.falconToDegrees(pivotMotor1.getSelectedSensorPosition(), Constants.pivotGearRatio);
  }

  public void setExtensionOutput(double output) {
    output = MathUtil.clamp(output, -1.0d, 1.0d);
    extensionMotor.set(output);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm extension current", extensionMotor.getOutputCurrent());
    SmartDashboard.putNumber("Arm extension position", extensionMotor.getSensorPosition());
  }
}
