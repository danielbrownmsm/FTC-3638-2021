package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
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
    
    private NormalizedColorSensor colorSensor;
    private NormalizedColorSensor otherColorSensor;

    private Telemetry telemetry;
    boolean willStopTurning = false;
    
    private PIDController distancePID = new PIDController(Constants.d_kP, Constants.d_kI, Constants.d_kD);
    private PIDController strafePID = new PIDController(Constants.s_kP, Constants.s_kI, Constants.s_kD);
    private PIDController headingPID = new PIDController(Constants.h_kP, Constants.h_kI, Constants.h_kD, -180, 180, true); // these two are continuous (max/min wrap around)
    private PIDController turningPID = new PIDController(Constants.t_kP, Constants.t_kI, Constants.t_kD, -180, 180, true);


    public DrivetrainSubsystem(Telemetry telemetry) {
        this.telemetry = telemetry;
        
        // aparently you have to do this in here?
        distancePID.setTolerance(0.1, 0.1);
        strafePID.setTolerance(0.1, 0.5);
        headingPID.setTolerance(1, 1);
        turningPID.setTolerance(5, 5);
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
        imuParameters.calibrationDataFile = "imu1_calib.json";
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT;
        imuParameters.accelerationIntegrationAlgorithm = null;
        
        /** Sensors */
        imu1 = map.get(BNO055IMU.class, "imu 1");
        imu1.initialize(imuParameters); // initialize the imu
  
        imu2 = map.get(BNO055IMU.class, "imu 2");
        imuParameters.calibrationDataFile = "imu2_calib.json";
        imu2.initialize(imuParameters); // initialize the imu
        
        colorSensor = map.get(NormalizedColorSensor.class, "rev_color_sensor");
        colorSensor.setGain(4);
        
        otherColorSensor = map.get(NormalizedColorSensor.class, "other_color");
        otherColorSensor.setGain(4);
    }
    
    public void addTelemetry() {
        telemetry.addData("Distance", getInches(getEncoderAverage()));
        telemetry.addData("Heading1", getYaw());
        telemetry.addData("Heading2", getOtherYaw());
        telemetry.addData("Ring Count", getRingCount());
        //telemetry.addData("==============");
    }
    
    /**
     * Gets the number of rings according to our distance sensors
     * 
     * @return the number of rings
     */
    public double getRingCount() {
        if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.INCH) < 20) {
            return 4;
        } else if (((DistanceSensor) otherColorSensor).getDistance(DistanceUnit.INCH) < 25) {
            return 1;
        }
        return 0;
    }
  
    /**
     * Gets if the imu is calibrated and ready-to-use or not
     * @return whether the IMU is calibrated or not
     */
     @Deprecated
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
     * Gets the average of the back encoders
     * @return the average of the encoders on the back side of the drivetrain
     */
    public double getStrafeEncoderAverage() {
        return (leftBack.getCurrentPosition() + rightFront.getCurrentPosition()) / 2;
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
     * Sets the last taken heading so we can drive straight in driveDistance and strafeDistance
     * @see driveDistance(), strafeDistance()
     */
    public void setHeading() {
        lastHeading = getYaw();
    }
  
    /**
     * Drives the robot on the last taken heading (so, straight) the distance given in inches, proportionally
     * @param inches how far you want to drive, in inches
     * 
     * @return if we have reached that distance or not
     */
    public boolean driveDistance(double inches) {
        inches = -inches;
        distancePID.setSetpoint(inches);
        if (distancePID.atSetpoint()) {
            arcadeDrive(0, 0);
            return true;
        } else {
            arcadeDrive(distancePID.calculate(getInches(getEncoderAverage()), System.currentTimeMillis()), headingPID.calculate((double) getYaw(), System.currentTimeMillis()));
            telemetry.addData("output", distancePID.calculate(getInches(getEncoderAverage()), System.currentTimeMillis()));
            telemetry.addData("setpoint", distancePID.getSetpoint());
            telemetry.addData("error", distancePID.getError());
            
            return false;
        }
    }
  
    /**
     * Strafes us a distance left/right, trying (hopefully) to keep us straight
     * @param inches how far you want to strafe, in inches
     * @return if we've reached the distance or not
     */
    public boolean strafeDistance(double inches) {
        strafePID.setSetpoint(inches);
        if (strafePID.atSetpoint()) {
            arcadeDrive(0, 0);
            return true;
        } else {
            arcadeDrive(strafePID.calculate(getInches(getEncoderAverage()), System.currentTimeMillis()), headingPID.calculate((double) getYaw(), System.currentTimeMillis()));
            return false;
        }
    }
  
    /**
     * Gets the current gyro yaw angle (so, turning stuff)
     * @return the yaw of the gyro
     */
    @Deprecated
    public float getYaw() { 
        float tempHeading = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return tempHeading;
    }
    
    /**
     * Gets the current average of the gyro yaw angles (so, turning stuff)
     * @return the yaw of the gyros
     */
     @Deprecated
    public float getOtherYaw() { 
        float tempHeading = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;
        //float otherTempHeading = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;
        //return (tempHeading + otherTempHeading) / 2; // average
        return tempHeading;
    }
  
    /**
     * Turns to a heading, in place. Make sure to call setHeading() before this
     * @param heading the heading you want to turn to, relative to the robot
     */
    /*public boolean turnToHeading(float heading) {
        if (lastHeading + heading - getYaw() < 3) { // if the error is less than our threshold
            arcadeDrive(0, (lastHeading + heading - getYaw()) * Constants.turn_turn_kP); // turn in-place, proportionally
            telemetry.addData("reading", lastHeading + heading - getYaw());
            return false;
        } else {
            //arcadeDrive(0, 0);  
            //return true; // we have reached that heading
            return false;
        }
    }*/
    
    /*public boolean turnToHeading(float heading) {
        dataHeading = heading;
        if (Math.abs(lastHeading + heading - getYaw()) < 3) { // if the error is less than our threshold
            arcadeDrive(0, (lastHeading + heading - getYaw()) * Constants.turn_turn_kP); // turn in-place, proportionally
            telemetry.addData("reading", lastHeading + heading - getYaw());
            telemetry.update();
            return false;
        } else {
            //arcadeDrive(0, 0);  
            //return true; // we have reached that heading
            return false;
        }
    }*/
    
    public boolean turnToHeading(float heading) {
        // goal + last = 183
        // > 180 so -180 = 3
        // then 180 - that * -1
        float tempGoal = heading + lastHeading;
        if (tempGoal < -180) {
            tempGoal += 180;
            tempGoal = 180 - tempGoal;
            tempGoal *= -1;
        } else if (tempGoal > 180) {
            tempGoal -= 180;
            tempGoal = 180 - tempGoal;
            tempGoal *= -1;
        }
        
        telemetry.addData("last heading", lastHeading);
        telemetry.addData("heading", heading);
        telemetry.addData("temp goal", tempGoal);
        
        if (Math.abs(tempGoal - getYaw()) < 2) {
            if (willStopTurning) {
                return true;
            }
            willStopTurning = true;
            return false;
        } else if (tempGoal < getYaw()) {
            arcadeDrive(0, -0.2);
            willStopTurning = false;

            return false;
        } else if (tempGoal > getYaw()) {
            arcadeDrive(0, 0.2);
            willStopTurning = false;

            return false;
        } else {
            return true; // how the h*** did we get here?
        }
    }
    
    /* For now only handles absolute turns, not relative (like turn to 90 degrees vs turn 30 degrees right where you're already at like 20 so you end up at 50 instead of 30) TODO XXX BUG FIXME */
    /*public boolean turnToHeading(double heading) {
        turningPID.setSetpoint(heading);
        if (turningPID.atSetpoint()) {
            arcadeDrive(0, 0);
            return true;
        } else {
            arcadeDrive(0, turningPID.calculate((double) getYaw(), System.currentTimeMillis()));
            telemetry.addData("output", turningPID.calculate(getInches(getEncoderAverage()), System.currentTimeMillis()));
            telemetry.addData("setpoint", turningPID.getSetpoint());
            telemetry.addData("error", turningPID.getError());
            
            //if (Math.abs(Math.abs(heading) - Math.abs(turningPID.getError())) < 1.5) { // lol idk if it works it works
            // screw this
            //    return true;
            //}
            return false;
        }
    }*/
  
    /**
     * Drives us in tele-op all mecanum-y
     * @param leftStickX controls the strafing 
     * @param leftStickY controls forwards/backwards
     * @param rightStickX controls the turning
     */
    public void driveTeleOp(float leftStickX, float leftStickY, float rightStickX) {
        if (Math.abs(leftStickX) < Constants.deadband) {
            leftStickX = 0;
        }
        if (Math.abs(leftStickY) < Constants.deadband) {
            leftStickY = 0;
        }
        if (Math.abs(rightStickX) < Constants.deadband) {
            rightStickX = 0;
        }
        
        leftFront.setPower(leftStickY - leftStickX - rightStickX);
        rightFront.setPower(leftStickY + leftStickX + rightStickX);
        leftBack.setPower(leftStickY + leftStickX - rightStickX);
        rightBack.setPower(leftStickY - leftStickX + rightStickX);
    }
    
    /**
     * Sets the zeroPowerBehavior of all the drivetrain motors
     * @param b the DcMotor.ZeroPowerBehavior you want all the motors set to
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior b) {
        leftFront.setZeroPowerBehavior(b);
        rightFront.setZeroPowerBehavior(b);
        leftBack.setZeroPowerBehavior(b);
        rightBack.setZeroPowerBehavior(b);
    }
}