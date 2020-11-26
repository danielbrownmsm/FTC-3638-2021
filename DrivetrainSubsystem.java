public class DrivetrainSubsystem {
  private DcMotor leftFront;
  private DcMotor rightFront;
  private DcMotor leftBack;
  private DcMotor rightBack;
  
  private Servo armServo;
  private Servo clawServo;
  
  private HardwareMap map;
  
  public DrivetrainSubsystem() {
  }
  
  /** Idk what this is */
  public void init(HardwareMap map_) {
    this.map = map_;
    
    leftFront
    rightFront
    leftBack
    rightBack
    
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
  }
}
