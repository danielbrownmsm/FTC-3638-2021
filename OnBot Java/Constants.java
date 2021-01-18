package org.firstinspires.ftc.teamcode;

public class Constants {
  public static final double drive_kP = 0.3;
  public static final double turn_kP = 0.05; // need to tune
  public static final double arm_kP = 0.01;
  
  public static final double headingThreshold = 0.1; // tune
  public static final double TICKS_PER_REVOLUTION = 512;
  public static final double WHEEL_DIAMETER = 4;
  public static final int driftDistance = 20; // tune

  public static final double wobbleServoUp = 0.3; // 0.4
  public static final double wobbleServoLeft = 0.95; // 0.85
  public static final double wobbleServoRight = 0.2; // 0.2
  public static final double wobbleServoDown = 0.95;
  
  public static final double wobbleClawOpen = 0.5;
  public static final double wobbleClawClosed = 1;
}
