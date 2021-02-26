package org.firstinspires.ftc.teamcode;

public class Constants {
  public static final double drive_kP = 0.3;
  public static final double strafe_kP = 0.2;
  public static final double turn_kP = 0.007; // need to tune
  
  public static final double shoot_kP = 0.001;
  public static final double targetRPM = -2000;
  public static final double accelLimit = 0.1;

  public static final double headingThreshold = 0.1; // tune
  public static final double TICKS_PER_REVOLUTION = 512;
  public static final double WHEEL_DIAMETER = 4;
  public static final int driftDistance = 0; // tune

  public static final double wobbleServoUp = 0.6;
  public static final double wobbleServoLeft = 0.95;
  public static final double wobbleServoRight = 0.3;
  public static final double wobbleServoDown = 0.2;
  
  public static final double wobbleClawOpen = 0.9;
  public static final double wobbleClawClosed = 0.55;
}
