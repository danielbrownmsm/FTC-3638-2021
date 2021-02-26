package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class AtomicMotor {
    private DcMotorImplEx motor;
    
    public AtomicMotor(DcMotorImplEx motor) {
        this.motor = motor;
    }
    
    public void init() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder();
        motor.setPower(0);
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void setPower(double power) {
        motor.setPower(power);
    }
    
    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }
    
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }
    
    public double getVelocity() {
        return motor.getVelocity();
    }
    
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior b) {
        motor.setZeroPowerBehavior(b);
    }
    
}