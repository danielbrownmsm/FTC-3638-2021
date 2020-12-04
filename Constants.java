package org.firstinspires.ftc.teamcode;

public class Constants {
  public static final double drive_kP = 0.3; // need to tune
  public static final double turn_kP = 0.5; // need to tune
  public static final double arm_kP = 0.01; // need to tune
  
  public static final double headingThreshold = 0.1; // tune
  public static final double TICKS_PER_REVOLUTION = 512; // change to actual value
  public static final double WHEEL_DIAMETER = 4; // change to actual value
  public static final int driftDistance = 20; // robot kinda drifts for 20 inches cause momentum

  public static final double wobbleServoUp = 0.4;
  public static final double wobbleServoLeft = 0.85;
  public static final double wobbleServoRight = 0.2; // change later
  public static final double wobbleServoDown = 0;
}
