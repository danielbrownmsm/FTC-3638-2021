package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

/** A class to represent our robot */
public class DrivetrainSubsystem {
    /** Motors */
    private AtomicMotor leftFront;
    private AtomicMotor rightFront;
    private AtomicMotor leftBack;
    private AtomicMotor rightBack;
  
    /** Sensors */
    private BNO055IMU.Parameters imuParameters;
    private BNO055IMU imu1;
    private BNO055IMU imu2;
    private float lastHeading = 0;

    private Telemetry telemetry;

    public DrivetrainSubsystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
  
    /**
     * Initializes the robot with the given hardware map
     * @param map_ the HardwareMap for the op-mode so we can instantiate our sensors and actuators
     */
    public void init(HardwareMap map) {  
        /** Create all our motors */
        leftFront = new AtomicMotor(map.get(DcMotorImplEx.class, "front_left"));
        rightFront = new AtomicMotor(map.get(DcMotorImplEx.class, "front_right"));
        leftBack = new AtomicMotor(map.get(DcMotorImplEx.class, "back_left"));
        rightBack = new AtomicMotor(map.get(DcMotorImplEx.class, "back_right"));
        
        /** Use the method in AtomicMotor to initialize the motors with the values we normally use */
        leftFront.init();
        rightFront.init();
        leftBack.init();
        rightBack.init();
  
        /** Set motor directions (ones on right side are reversed */
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        
        /** Friggin undocumented bits */
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.calibrationDataFile = "AdafruitIMUCalibration.json.json";
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT;
        imuParameters.accelerationIntegrationAlgorithm = null;
        
        /** Sensors */
        imu1 = map.get(BNO055IMU.class, "imu 1");
        imu1.initialize(imuParameters); // initialize the imu
  
        imu2 = map.get(BNO055IMU.class, "imu 2");
        imu2.initialize(imuParameters); // initialize the imu
    }
  
    /**
     * Gets if the imu is calibrated and ready-to-use or not
     * @return whether the IMU is calibrated or not
     */
    public Boolean isGyroCalibrated() {
        return imu1.isSystemCalibrated() && imu2.isSystemCalibrated();
    }
  
    /**
     * Resets the encoders of the drivetrain
     */
    public void resetEncoders() {
        leftFront.resetEncoder();
        rightFront.resetEncoder();
        leftBack.resetEncoder();
        rightBack.resetEncoder();
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
        if (Math.abs(getInches(getEncoderAverage())) < Math.abs(inches)) { // if we haven't reached where we need to go
            arcadeDrive((inches - getInches(getEncoderAverage())) * -Constants.drive_kP, 
                        (lastHeading - getYaw()) * Constants.turn_kP); // drive there proportionally to how far away we are, and straight
            return false; // we haven't reached it yet
        } else {
            return true; // we're here!
        }
    }
  
    /**
     * Strafes us a distance left/right, trying (hopefully) to keep us straight
     * @param inches how far you want to strafe, in inches
     * @return if we've reached the distance or not
     */
    public boolean strafeDistance(double inches) {
        if (Math.abs(getInches(getEncoderAverage())) < Math.abs(inches)) { // if we haven't reached where we need to go
            driveTeleOp((float) 0.0, (float) ((inches - getInches(getEncoderAverage())) * Constants.drive_kP), 
                        (float) ((lastHeading - getYaw()) * (float) Constants.turn_kP)); // drive there proportionally to how far away we are, and straight
          return false; // we haven't reached it yet
        } else {
            driveTeleOp(0, 0, 0);
            return true; // we're here!
        }
    }
  
    /**
     * Gets the current gyro yaw angle (so, turning stuff)
     * @return the yaw of the gyro
     */
    public float getYaw() { 
        return imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;
    }
  
    /**
     * Turns to a heading, in place. Make sure to call setHeading() before this
     * @param heading the heading you want to turn to, relative to the robot
     */
    public boolean turnToHeading(float heading) {
        if (lastHeading + heading - getYaw() > Constants.headingThreshold) { // if the error is less than our threshold
            arcadeDrive(0, (lastHeading + heading - getYaw()) * Constants.turn_kP); // turn in-place, proportionally
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

    public void postImuStatus() {
        telemetry.addData("IMU calibrated: ", isGyroCalibrated());
    }
}