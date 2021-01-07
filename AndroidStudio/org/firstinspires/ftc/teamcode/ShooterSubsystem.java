package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.ftc11392.sequoia.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ShooterSubsystem extends Subsystem {
    private DcMotorImplEx shooterMotor;
    private PIDFController shooterPID = new PIDFController(Constants.shooter.kP, Constants.shooter.kI, Constants.shooter.kD, 0);
    private CRServoImplEx triggerServo;
    private double goalRPM = 0;

    @Override
    public void initialize(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorImplEx.class, "shooter");
        triggerServo = hardwareMap.get(CRServoImplEx.class, "trigger");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        triggerServo.setDirection(DcMotor.Direction.REVERSE);
        triggerServo.setPwmEnable();
        triggerServo.setPower(0);
    }

    public void setShooter(double rpm) {
        goalRPM = rpm;
    }

    public void setTrigger(double power) {
        triggerServo.setPower(power);
    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {
        shooterMotor.setPower(shooterPID.control(goalRPM, shooterMotor.getVelocity()));
    }

    @Override
    public void stop() {

    }
}
