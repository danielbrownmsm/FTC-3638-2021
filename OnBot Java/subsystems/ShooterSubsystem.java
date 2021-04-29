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
    private ServoImplEx shooterPos1;
    private ServoImplEx shooterPos2;
    private AtomicMotor shooter1;
    private AtomicMotor shooter2;

    private ServoImplEx intake;

    private Telemetry telemetry;

    public ShooterSubsystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap map) {
        shooter1 = new AtomicMotor(map.get(DcMotorImplEx.class, "shooter1"));
        shooter2 = new AtomicMotor(map.get(DcMotorImplEx.class, "shooter2"));

        shooter1.init();
        shooter2.init();
        
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        trigger = map.get(CRServoImplEx.class, "trigger");
        trigger.setPower(0);
        trigger.setDirection(DcMotorSimple.Direction.REVERSE);
        
        intake = map.get(ServoImplEx.class, "intake");

        shooterPos1 = map.get(ServoImplEx.class, "shooterPos1");
        shooterPos2 = map.get(ServoImplEx.class, "shooterPos1");
    }

    public void setAngle(double angle) {
        shooterPos1.setPosition(angle);
        shooterPos2.setPosition(angle);
    }
    
    public void setTrigger(double power) {
        trigger.setPower(power);
    }
    
    public void setIntake(double pos) {
        intake.setPosition(pos);
    }
    
    public double getVelocity() { // ticks / second
        return (shooter1.getVelocity() + shooter2.getVelocity()) / 2;
    }
    
    public void shoot(double power) {
        shooter1.setVelocity(power);
        shooter2.setVelocity(power);
    }
    
    public boolean isShooterGood() {
        return Math.abs(Constants.targetRPM - getVelocity()) < 100; // we want to be within 100 RPM of the setpoint
    }
    
    public void addTelemetry() {
        telemetry.addData("Shooter good", isShooterGood());
        telemetry.addData("Velocity", getVelocity());
        telemetry.addData("Intake Position", intake.getPosition());
        telemetry.addData("Shooter Position", shooterPos1.getPosition());
    }
}