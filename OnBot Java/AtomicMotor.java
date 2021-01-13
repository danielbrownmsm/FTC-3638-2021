package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class AtomicMotor extends DcMotorImplEx {
    public void init() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
        setPower(0);
    }

    public void resetEncoder() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}