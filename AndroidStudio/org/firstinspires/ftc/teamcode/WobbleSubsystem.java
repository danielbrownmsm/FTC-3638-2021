package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Const;

public class WobbleSubsystem extends Subsystem {
    private ServoImplEx armServo;
    private ServoImplEx clawServo;

    @Override
    public void initialize(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(ServoImplEx.class, "wobbleArm");
        clawServo = hardwareMap.get(ServoImplEx.class, "wobbleClaw");

        armServo.setPwmEnable();
        clawServo.setPwmEnable();
        armServo.setPosition(Constants.wobble.armInitPos);
        clawServo.setPosition(Constants.wobble.clawInitPos);
    }

    public void setClaw(double position) {
        clawServo.setPosition(position);
    }

    public void setArm(double position) {
        armServo.setPosition(position);
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
        armServo.setPwmDisable();
        clawServo.setPwmDisable();
    }
}
