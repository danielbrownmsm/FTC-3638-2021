package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class CustomRobot {
  /** Motors */
  private DcMotor leftFront;
  private DcMotor rightFront;
  private DcMotor leftBack;
  private DcMotor rightBack;
  private DcMotor armMotor;
  
  /** Servos */
  private Servo armServo;
  private Servo clawServo;
  private Servo ringServo;
  
  /** Sensors + control stuff */
  private BNO055IMU.Parameters imuParameters;
  private BNO055IMU imu1;
  private double drive_kP = 0.05; // need to tune
  private double turn_kP = 0.05; // need to tune
  private double headingThreshold = 0.1; // tune
  
  private double TICKS_PER_REVOLUTION = 1; // change to actual value
  private double WHEEL_DIAMETER = 4; // change to actual value
  
  private HardwareMap map;
  
  public CustomRobot() {
  }
  
  /** Idk what this is */
  public void init(HardwareMap map_) {
    this.map = map_;
    
    /** Create all our motors */
    leftFront = map.get(DcMotor.class, "front_left");
    rightFront = map.get(DcMotor.class, "front_right");
    leftBack = map.get(DcMotor.class, "back_left");
    rightBack = map.get(DcMotor.class, "back_right");
    armMotor = map.get(DcMotor.class, "ring_arm");
    
    /** Set motor directions (ones on right side are reversed */
    leftFront.setDirection(DcMotor.Direction.FORWARD);
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    leftBack.setDirection(DcMotor.Direction.FORWARD);
    rightBack.setDirection(DcMotor.Direction.REVERSE);
    armMotor.setDirection(DcMotor.Direction.FORWARD);
    
    /** Set all motors to 0 */
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftBack.setPower(0);
    rightBack.setPower(0);
    armMotor.setPower(0);
    
    /** Set the mode of all the motors */
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
    /** Now for servos */
    armServo = map.get(Servo.class, "arm");
    clawServo = map.get(Servo.class, "claw");
    ringServo = map.get(Servo.class, "ring_claw");
    armServo.setPosition(0.5);
    clawServo.setPosition(1);
    ringServo.setPosition(1);
    
    /** Sensors */
    imu1 = map.get(BNO055IMU.class, "imu 1");
    imuParameters = new BNO055IMU.Parameters();
    imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imuParameters.mode = BNO055IMU.SensorMode.IMU;
    imuParameters.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT;
    imuParameters.accelerationIntegrationAlgorithm = null;
    imu1.initialize(imuParameters); // initialize the imu
  }
  
  /**
   * Gets if the imu is calibrated and ready-to-use or not
   */
  public Boolean isGyroCalibrated() {
    return imu1.isSystemCalibrated();
  }
  
  /**
   * Resets the encoders of the drivetrain
   */
  public void resetEncoders() {
    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
  }
  
  /**
   * Drive the robot arcade-style
   */
  public void arcadeDrive(double speed, double turn) {
    leftFront.setPower(speed + turn);
    leftBack.setPower(speed + turn);
    rightFront.setPower(speed - turn);
    rightBack.setPower(speed - turn);
  }
  
  /**
   * Gets the average of all encoders
   */
  public double getEncoderAverage() {
    return (getLeftEncoderAverage() + getRightEncoderAverage()) / 2;
  }
   
  /**
   * Gets the average of the left encoders
   */
  public double getLeftEncoderAverage() {
    return (leftFront.getCurrentPosition() + leftBack.getCurrentPosition()) / 2;
  }

  /**
   * Gets the average of the right encoders
   */
  public double getRightEncoderAverage() {
    return (rightFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;
  }
  
  /**
   * Given a tick count, returns that distance in inches
   */
  public double getInches(double ticks) {
    return ticks / TICKS_PER_REVOLUTION * WHEEL_DIAMETER * Math.PI;
  }

  /**
   * Drives the robot on the last taken heading (so, straight) the distance given in inches
   */
  public void driveDistance(double inches) {
    resetEncoders(); // reset encoders so we start fresh
    float lastHeading = getYaw(); // get the reference angle we will try to P to
    while (!(Math.abs(getEncoderAverage()) >= inches)) { // idk if this linear op mode thing will work here
      arcadeDrive((inches - getInches(getEncoderAverage())) * drive_kP, 
                  (lastHeading - getYaw()) * turn_kP);
    }
  }
  
  /**
   * Gets the current gyro yaw angle (so, turning stuff)
   */
  public float getYaw() {
    return imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
  }
  
  /**
   * Turns to a heading, in place
   */
  public void turnToHeading(float heading) {
    float newHeading = getYaw() + heading; // get the new heading (so that all params passed to this function can be relative)
    while (!(newHeading - getYaw() <= headingThreshold)) { // until the error is less than our threshold
      arcadeDrive(0, (newHeading - getYaw()) * turn_kP); // turn in-place
    }
  }

  /**
   * Drives us in tele-op all mecanum-y
   * @param leftStickX controls the strafing 
   * @param leftStickY controls forwards/backwards
   * @param rightStickX controls the turning
   */
  public void driveTeleOp(float leftStickX, float leftStickY, float rightStickX) {
    leftFront.setPower(leftStickY - leftStickX - rightStickX);
    rightFront.setPower(leftStickY + leftStickX + rightStickX);
    leftBack.setPower(leftStickY + leftStickX - rightStickX);
    rightBack.setPower(leftStickY - leftStickX + rightStickX);
  }

  /**
   * Sets the wobble goal claw to either 1 or 0
   * depending on if you want it open or not
   * @param open if you want the claw open or not
   */
  public void setWobbleClaw(boolean open) {
    if (open) {
      clawServo.setPosition(0);
    } else {
      clawServo.setPosition(1);
    }
  }

  /**
   * Sets the ring claw to either 0 or 1 depending on open
   * @param open if you want the claw open or not
   */
  public void setRingClaw(boolean open) {
    if (open) {
      ringServo.setPosition(0);
    } else {
      ringServo.setPosition(1);
    }
  }

  /**
   * Sets the arm servo to the position you give it
   * @param position the position you want the arm servo to go to
   */
  public void setArm(double position) {
    armServo.setPosition(position);
  }

  public void setRingArm(double position) {
    armMotor.setPower(position - ringArmPosition());
  }

  public float ringArmPosition() {
    return armMotor.getCurrentPosition();
  }
}
