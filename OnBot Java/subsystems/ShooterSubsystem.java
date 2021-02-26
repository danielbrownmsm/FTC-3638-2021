package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import org.firstinspires.ftc.robotcore.external.Const;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {
    private CRServoImplEx trigger;
    private AtomicMotor shooter;
    private double lastOutput = 0;
    private double lastTime = 0;

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

    public void shoot(double millis) {
        //double output = (Constants.targetRPM + shooter.getVelocity()) * Constants.shoot_kP;
        //if (Math.abs((output - lastOutput) / (millis - lastTime)) > Constants.accelLimit) {
        //    shooter.setPower(lastOutput + Constants.accelLimit);
        //    lastOutput = lastOutput + Constants.accelLimit;
        //    lastTime = millis;
        //} else {
        //    shooter.setPower(output);
        //    lastOutput = output;
        //    lastTime = millis;
        //}
        //shooter.setPower((Constants.targetRPM - -shooter.getVelocity()) * Constants.shoot_kP);
        //double error = Constants.targetRPM - -shooter.getVelocity();
        //if (error < 0) {
        //    shooter.setPower(0.5);
        //} else {
        //    shooter.setPower(error * Constants.shoot_kP);
        //}
        if (shooter.getVelocity() > Constants.targetRPM) {
            shooter.setPower(1);
            //trigger.setPower(0);
        } else {
            shooter.setPower(0);
            //trigger.setPower(1);
        }
    }
    
    public void stop() {
        shooter.setPower(0);
    }
    
    public double getSpeed() {
        return shooter.getVelocity();
    }
    
    public void setTrigger(double power) {
        trigger.setPower(power);
    }

}