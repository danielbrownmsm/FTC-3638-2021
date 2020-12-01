public class Robot {
  private DcMotor leftFront;
  private DcMotor rightFront;
  private DcMotor leftBack;
  private DcMotor rightBack;
  
  private Servo armServo;
  private Servo clawServo;
  
  private BNO055IMU.Parameters imuParameters;
  private double driveKp = 0.05; // need to tune
  private double turnKp = 0.05; // need to tune
  
  private HardwareMap map;
  
  public RObot() {
  }
  
  /** Idk what this is */
  public void init(HardwareMap map_) {
    this.map = map_;
    
    /** Create all our motors */
    leftFront = map.get(DcMotor.class, "left_front");
    rightFront = map.get(DcMotor.class, "right_front");
    leftBack = map.get(DcMotor.class, "left_back");
    rightBack = map.get(DcMotor.class, "right_back");
    
    /** Set motor directions (ones on right side are reversed */
    leftFront.setDirection(DcMotor.Direction.FORWARD);
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    leftBack.setDirection(DcMotor.Direction.FORWARD);
    rightBack.setDirection(DcMotor.Direction.REVERSE);
    
    /** Set all motors to 0 */
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftBack.setPower(0);
    rightBack.setPower(0);
    
    /** Set the mode of all the motors */
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
    /** Now for servos */
    armServo = map.get(Servo.class, "arm");
    clawServo = map.get(Servo.class, "claw");
    armServo.setPosition(0);
    clawServo.setPosition(1);
    
    /** Sensors */
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
    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODERS);
    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODERS);
    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODERS);
    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODERS);
    
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
  }
  
  /**
   * Drive the robot arcade-style
   */
  public void arcadeDrive(double speed, double turn) {
    leftFront.setPower(drive + turn);
    leftBack.setPower(drive + turn);
    rightFront.setPower(drive - turn);
    rightBack.setPower(drive - turn);
  }
  
  /**
   * Gets the average of all encoders
   */
  public double getEncoderAverage() {
    return (getLeftEncoderAverage() + getRightEncoderAverage) / 2;
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
  private void driveDistance(double inches) {
    resetEncoders(); // reset encoders so we start fresh
    float lastHeading = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle; // get the reference angle we will try to P to
    while (!(Math.abs(getEncoderAverage()) >= inches) || linearOpMode.isStopRequested()) { // idk if this linear op mode thing will work here
      arcadeDrive((inches - getInches(getEncoderAverage())) * drive_kP, 
                  (lastHeading - imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) * turn_kP);
    }
  }
  
}
