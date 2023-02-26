package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Led {
  
  public enum PatternID {
	  OFF(0, "off"),
	  VIOLET(0.91, "Solid Violet"),
	  YELLOW(0.69, "Solid Yellow");
	private final double m_pwmVal;
	private final String m_patternName;

	private PatternID(double pwmVal, String patternName) {
		m_pwmVal = pwmVal;
		m_patternName = patternName;
	}

	private double getVal() {
		return m_pwmVal;
	}

	public String getName() {
		return m_patternName;
	}
  }

  private static final Spark m_blinkin = new Spark(Constants.revBlinkinPort);

  public static void set(PatternID pattern) {
	m_blinkin.set(pattern.getVal());
  }
}
