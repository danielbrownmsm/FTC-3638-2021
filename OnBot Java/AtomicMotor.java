package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class AtomicMotor extends DcMotorImplEx {
    public AtomicMotor(DcMotorController controller, int port, DcMotor.Direction direction, MotorConfigurationType conf) {
        super(controller, port, direction, conf);
    }
    
    public void init() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder();
        setPower(0);
    }

    public void resetEncoder() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}