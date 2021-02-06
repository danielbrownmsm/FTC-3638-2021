package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {
    private CRServoImplEx trigger;
    private AtomicMotor shooter;

    private Telemetry telemetry;

    public ShooterSubsystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap map) {
        shooter = new AtomicMotor(map.get(DcMotorImplEx.class, "shooter"));
        shooter.init();
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        trigger = map.get(CRServoImplEx.class, "trigger");
        trigger.setPower(0);
        trigger.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void warmUp(double power) {
        shooter.setPower(power);
    }

    public void shoot(double power) {
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setPower(power);
        trigger.setPower(power);
    }
    
    public void setTrigger(double power) {
        trigger.setPower(power);
    }

} 