package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import org.firstinspires.ftc.robotcore.external.Const;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {
    private CRServoImplEx trigger;
    private ServoImplEx intake;
    private AtomicMotor shooter;

    private Telemetry telemetry;
    
    private double lastPosition;
    private double currentPosition;
    private double lastTime;
    private double currentTime;
    private double currentVelocity;
    private double lastVelocity;

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
        
        intake = map.get(ServoImplEx.class, "intake");
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
    
    public void setIntake(double pos) {
        intake.setPosition(pos);
    }
    
    public double getVelocity(double currentTime) { // ticks / second
        currentPosition = shooter.getCurrentPosition();
        currentVelocity = (currentPosition - lastPosition) / (currentTime - lastTime); // both should eval to minus so in the end its a positive
        
        lastPosition = currentPosition;
        lastTime = currentTime;
        lastVelocity = currentVelocity;
        
        return currentVelocity / 537.6 * -60; // should be Rots / Sec * 60 so RPM
    }
    
    public void shootPID(double currTime) {
        double power = (Constants.targetRPM - getVelocity(currTime)) * Constants.shoot_kP;
        if (power < 0) {
            power = 0;
        }
        shooter.setPower(power);
    }
    
    public boolean isShooterGood() {
        return Math.abs(Constants.targetRPM - lastVelocity) < 10; // we want to be within 10 RPM of the setpoint
    }

} 