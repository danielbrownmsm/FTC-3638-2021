package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

/** A class to represent our robot */
public class CustomRobot {
  /** Motors */
  private DcMotor leftFront;
  private DcMotor rightFront;
  private DcMotor leftBack;
  private DcMotor rightBack;
  private DcMotor ringArm;
  
  /** Servos */
  private Servo wobbleArm;
  private Servo wobbleClaw;
  private Servo ringClaw;
  
  /** Sensors + control stuff */
  private BNO055IMU.Parameters imuParameters;
  private BNO055IMU imu1;
  private int ringArmTargetPosition = 10;
  
  private HardwareMap map;
  
  public CustomRobot() {
  }
  
  /**
   * Initializes the robot with the given hardware map
   * @param map_ the HardwareMap for the op-mode so we can instantiate our sensors and actuators
   */
  public void init(HardwareMap map_) {
    this.map = map_;
    
    /** Create all our motors */
    leftFront = map.get(DcMotor.class, "front_left");
    rightFront = map.get(DcMotor.class, "front_right");
    leftBack = map.get(DcMotor.class, "back_left");
    rightBack = map.get(DcMotor.class, "back_right");
    ringArm = map.get(DcMotor.class, "ring_arm");
    
    /** Set motor directions (ones on right side are reversed */
    leftFront.setDirection(DcMotor.Direction.FORWARD);
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    leftBack.setDirection(DcMotor.Direction.FORWARD);
    rightBack.setDirection(DcMotor.Direction.REVERSE);
    ringArm.setDirection(DcMotor.Direction.FORWARD);
    
    /** Set all motors to 0 */
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftBack.setPower(0);
    rightBack.setPower(0);
    ringArm.setPower(0);
    
    /** Set the mode of all the motors */
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    ringArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ringArm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
    /** Now for servos */
    wobbleArm = map.get(Servo.class, "arm");
    wobbleClaw = map.get(Servo.class, "claw");
    ringClaw = map.get(Servo.class, "ring_claw");
    
    wobbleClaw.setPosition(1);
    ringClaw.setPosition(1);
    
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
   * @return whether the IMU is calibrated or not
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
   * @param speed the power you want to drive forwards/reverse at
   * @param turn the amount of turn you want to have
   */
  public void arcadeDrive(double speed, double turn) {
    leftFront.setPower(speed + turn);
    leftBack.setPower(speed + turn);
    rightFront.setPower(speed - turn);
    rightBack.setPower(speed - turn);
  }
  
  /**
   * Gets the average of all encoders
   * @return the average of the left and right encoders on the drivetrain
   */
  public double getEncoderAverage() {
    return (getLeftEncoderAverage() + getRightEncoderAverage()) / 2;
  }
   
  /**
   * Gets the average of the left encoders
   * @return the average of the encoders on the left side of the drivetrain
   */
  public double getLeftEncoderAverage() {
    return (leftFront.getCurrentPosition() + leftBack.getCurrentPosition()) / 2;
  }

  /**
   * Gets the average of the right encoders
   * @return the average of the encoders on the right side of the drivetrain
   */
  public double getRightEncoderAverage() {
    return (rightFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;
  }
  
  /**
   * Given a tick count, returns that distance in inches
   * @param ticks the number of ticks of the encoder
   * @return the distance that that number represents, in inches
   */
  public double getInches(double ticks) {
    return ticks / Constants.TICKS_PER_REVOLUTION * Constants.WHEEL_DIAMETER * Math.PI;
  }

  /**
   * Sets the last taken heading so we can drive straight in driveDistance
   */
  public void setHeading() {
    lastHeading = getYaw();
  }

  /**
   * Drives the robot on the last taken heading (so, straight) the distance given in inches
   * @param inches how far you want to drive, in inches
   * @return if we have reached that distance or not
   */
  public boolean driveDistance(double inches) {
    if (Math.abs(getInches(getEncoderAverage())) <= inches) { // if we haven't reached where we need to go
      arcadeDrive((inches - getInches(getEncoderAverage())) * Constants.drive_kP, 
                  (lastHeading - getYaw()) * Constants.turn_kP); // drive there proportionally to how far away we are, and straight
      return false; // we haven't reached it yet
    } else {
      return true; // we're here!
    }
  }
  
  /**
   * Gets the current gyro yaw angle (so, turning stuff)
   * @return the yaw of the gyro
   */
  public float getYaw() {
    return imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
  }
  
  /**
   * Turns to a heading, in place. Make sure to call setHeading() before this
   * @param heading the heading you want to turn to, relative to the robot
   */
  public boolean turnToHeading(float heading) {
    if (lastHeading + heading - getYaw() >= Constants.headingThreshold)) { // if the error is less than our threshold
      arcadeDrive(0, (newHeading - getYaw()) * Constants.turn_kP); // turn in-place, proportionally
      return false;
    } else {
      return true; // we have reached that heading
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
      wobbleClaw.setPosition(0);
    } else {
      wobbleClaw.setPosition(1);
    }
  }

  /**
   * Sets the ring claw to either 0 or 1 depending on open
   * @param open if you want the claw open or not
   */
  public void setRingClaw(boolean open) {
    if (open) {
      ringClaw.setPosition(0);
    } else {
      ringClaw.setPosition(0.2);
    }
  }
  
  /**
   * "Kicks" the ring claw outwards to knock any rings off of our pointy part
   */
  public void ringClawKick() {
    ringClaw.setPosition(1);
  }

  /**
   * Sets the arm servo to the position you give it
   * @param position the position you want the arm servo to go to
   */
  public void setWobbleArm(double position) {
    wobbleArm.setPosition(position);
  }
  
  /**
   * Sets the ring arm target it should P to
   * @param target the count of encoder ticks the ring arm should try to stay at
   */
  public void setRingArmTarget(int target) {
    ringArmTargetPosition = target;
  }
  
  /**
   * Increments the ring arm target by the given value
   * @param val the number of ticks (can be negative) you want to increment the ring arm target by
   */
  public void incrementRingArm(int val) {
    ringArmTargetPosition += val;
  }
  
  /**
   * Should be called once every loop (or, in the case of auto, very often)
   * Does things like telemetry (although not right now) and other stuff we want to run often,
   * like making sure the arm stays where it's supposed to
   */
  public void periodic() {
    ringArm.setPower((ringArmTargetPosition - ringArm.getCurrentPosition()) * Constants.arm_kP);
  }
  
  /**
   * Sets the zero power behavior (either BRAKE or FLOAT) of the drivetrain motors
   * @param behavior a DcMotor.ZeroPowerBehavior
   */
  public void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
    leftFront.setZeroPowerBehavior(behavior);
    rightFront.setZeroPowerBehavior(behavior);
    leftBack.setZeroPowerBehavior(behavior);
    rightBack.setZeroPowerBehavior(behavior);
  }
}
