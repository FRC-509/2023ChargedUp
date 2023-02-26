package frc.robot.util;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public final class PIDConstants {
  public final double kP;
  public final double kI;
  public final double kD;
  public final double kF;

  public PIDConstants(double kP, double kI, double kD, double kF) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
  }

  public void configureTalonFX(TalonFX talon) {
    talon.config_kP(0, kP);
    talon.config_kI(0, kI);
    talon.config_kD(0, kD);
    talon.config_kF(0, kF);
  }
}