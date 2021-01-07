package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivetrainSubsystem extends Subsystem {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private BNO055IMU imu1;
    private BNO055IMU imu2;

    private BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
    imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imuParameters.mode = BNO055IMU.SensorMode.IMU;
    imuParameters.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT;


    public DrivetrainSubsystem() {}

    @Override
    public void initialize(HardwareMap map) {
        frontLeft = map.get(DcMotor.class, "frontLeft");
        frontRight = map.get(DcMotor.class, "frontRight");
        backLeft = map.get(DcMotor.class, "backLeft");
        backRight = map.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu1 = map.get(BNO055IMU.class, "imu 1");
        imu1.initialize(imuParameters);

        imu2 = map.get(BNO055IMU.class, "imu 2");
        imu2.initialize(imuParameters);
    }

    public double getEncoderAverage() {
        return (getLeftAverage() + getRightAverage()) / 2
    }

    public double getLeftAverage() {
        return (backRight.getCurrentPosition() + backLeft.getCurrentPosition()) / 2;
    }

    public double getRightAverage() {
        return (frontRight.getCurrentPosition() + backRight.getCurrentPosition()) / 2;
    }

    /**
     * Drives us in tele-op all mecanum-y
     * @param leftStickX controls the strafing 
     * @param leftStickY controls forwards/backwards
     * @param rightStickX controls the turning
     */
    public void driveTeleOp(float leftStickX, float leftStickY, float rightStickX) {
        frontLeft.setPower(leftStickY - leftStickX - rightStickX);
        frontRight.setPower(leftStickY + leftStickX + rightStickX);
        backLeft.setPower(leftStickY + leftStickX - rightStickX);
        backRight.setPower(leftStickY - leftStickX + rightStickX);
    }

    public double getYaw() {
      return (imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle + imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) / 2;
    }

    public Boolean driveDistance(double ticks) {
        //TODO
        //plz add Roadrunner
        // b/c they can do so much better than I
    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {

    }

    @Override
    public void stop() {

    }

}
