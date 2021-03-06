package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WobbleSubsystem {
    private ServoImplEx wobbleArm;
    private ServoImplEx wobbleClaw;
    private Telemetry telemetry;

    public WobbleSubsystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap map) {
        wobbleArm = map.get(ServoImplEx.class, "wobbleArm");
        wobbleClaw = map.get(ServoImplEx.class, "wobbleClaw");
    }

    public void setClaw(double position) {
        wobbleClaw.setPosition(position);
    }

    public void setArm(double position) {
        wobbleArm.setPosition(position);
    }

    public void setArmStatus(boolean enabled) {
        if (enabled) {
            wobbleArm.setPwmEnable();
        } else {
            wobbleArm.setPwmDisable();
        }
    }

    public void powerSave() {
        wobbleArm.setPwmDisable();
        wobbleClaw.setPwmDisable();
    }

    public void unPowerSave() {
        wobbleArm.setPwmEnable();
        wobbleClaw.setPwmEnable();
    }

}
    