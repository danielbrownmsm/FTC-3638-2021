package org.firstinspires.ftc.teamcode;

public class Constants {
  // PID vals
  public static final double d_kP = 0.03;
  public static final double d_kI = 0.00000001;
  public static final double d_kD = 0.000000001;

  public static final double s_kP = 0.1;
  public static final double s_kI = 0.01;
  public static final double s_kD = 0.001;

  public static final double h_kP = 0.001;
  public static final double h_kI = 0.0001;
  public static final double h_kD = 0.001;
  
  public static final double t_kP = 0.01;
  public static final double t_kI = 0.0001;
  public static final double t_kD = 0.01;

//  public static final double shoot_kP = 0.0001;

  public static final double headingThreshold = 0.1; // tune
  public static final double TICKS_PER_REVOLUTION = 512;
  public static final double WHEEL_DIAMETER = 4;
  public static final int driftDistance = 0; // tune
  public static final double deadband = 0.05;

  public static final double wobbleServoUp = 0.6;
  public static final double wobbleServoLeft = 0.95;
  public static final double wobbleServoRight = 0.3;
  public static final double wobbleServoDown = 0.2;
  
  public static final double wobbleClawOpen = 1;
  public static final double wobbleClawClosed = 0.3;
  
  public static final double intakeUp = 0;
  public static final double intakeDown = 0.7;
  public static final double intakeNeutral = 0.2;
  public static final double intakeAuto = 0.05;
  
  public static final double targetRPM = 1100;
}