package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
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
        shooter = map.get(AtomicMotor.class, "shooter");
        shooter.init();
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        trigger = map.get(CRServoImplEx.class, "trigger");
        trigger.setPower(0);
    }

    public void warmUp(double power) {
        shooter.setPower(power);
    }

    public void shoot(double power) {
        shooter.setPower(power);
        trigger.setPower(1);
    }

} 