package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
@Autonomous(name = "DriveToLineAuto (Blocks to Java)", group = "")
public class DriveToLineAuto extends LinearOpMode {
  private Servo claw;
  private Servo arm;
  private DcMotor front_left;
  private DcMotor front_right;
  private DcMotor back_left;
  private DcMotor back_right;
  private BNO055IMU imu1;
  double drive_kP;
  double turn_kP;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    BNO055IMU.Parameters imuParameters;
    imuParameters = new BNO055IMU.Parameters();
    claw = hardwareMap.get(Servo.class, "claw");
    arm = hardwareMap.get(Servo.class, "arm");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    front_right = hardwareMap.get(DcMotor.class, "front_right");
    back_left = hardwareMap.get(DcMotor.class, "back_left");
    back_right = hardwareMap.get(DcMotor.class, "back_right");
    imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");
    telemetry.addData("Status", "Setting up Motors...");
    telemetry.update();
    claw.setPosition(1);
    arm.setPosition(0);
    front_left.setDirection(DcMotorSimple.Direction.FORWARD);
    front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    back_left.setDirection(DcMotorSimple.Direction.FORWARD);
    back_right.setDirection(DcMotorSimple.Direction.REVERSE);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
    // init our IMU
    imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imuParameters.mode = BNO055IMU.SensorMode.IMU;
    imuParameters.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT;
    imuParameters.accelerationIntegrationAlgorithm = null;
    imu1.initialize(imuParameters);
    if (!imu1.isSystemCalibrated()) {
      telemetry.addData("Stauts", "IMU not calibrated!");
      telemetry.update();
    }
    drive_kP = 0.05;
    turn_kP = 0.05;
    telemetry.addData("Status", "Waiting for start...");
    telemetry.update();
    waitForStart();
    driveDistance(100);
  }
  /**
   * Gets the average of all four encoders
   */
  private double getEncoderAverage() {
    return (front_left.getCurrentPosition() + front_right.getCurrentPosition() + back_left.getCurrentPosition() + back_right.getCurrentPosition()) / 4;
  }
  /**
   * Returns the sign of the variable passed to it
   */
  private double getSign(double x) {
    if (x < 0) {
      return -1;
    }
    return 1;
  }
  /**
   * Arcade drive, copied from WPILibJ's Differential Drive
   */
  private void arcadeDrive(double speed, double rotation) {
    double maxInput;
    // This was copied from WPILibJ (the DifferentialDrive class arcadeDrive() function, so blame them if something is wrong
    maxInput = getSign(speed) * JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(speed), Math.abs(rotation)));
    if (speed >= 0) {
      if (rotation >= 0) {
        setLeftMotors(maxInput);
        setRightMotors(speed - rotation);
      } else {
        setLeftMotors(speed + rotation);
        setRightMotors(maxInput);
      }
    } else {
      if (speed >= 0) {
        setLeftMotors(speed + rotation);
        setRightMotors(maxInput);
      } else {
        setLeftMotors(maxInput);
        setRightMotors(speed - rotation);
      }
    }
  }
  /**
   * Drives the distance given based off of encoder counts
   */
  private void driveDistance(double inches) {
    float lastHeading;
    lastHeading = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    while (!(Math.abs(getEncoderAverage()) >= inches)) {
      arcadeDrive((inches - getEncoderAverage()) * drive_kP, (lastHeading - imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) * turn_kP);
    }
  }
  /**
   * Sets the power of the left motors
   */
  private void setLeftMotors(double power) {
    front_left.setPower(1);
    back_left.setPower(1);
  }
  /**
   * Sets the power of the right motors
   */
  private void setRightMotors(double power) {
    front_right.setPower(1);
    back_right.setPower(1);
  }
}
