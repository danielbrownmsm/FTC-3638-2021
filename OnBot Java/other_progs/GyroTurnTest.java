package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Const;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "GyroTurnTest", group = "")
public class GyroTurnTest extends LinearOpMode {
    private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(telemetry);

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode()  {
        drivetrain.init(hardwareMap);

        waitForStart();
    
        /** Strafe to pick up the other wobble goal */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while (!drivetrain.turnToHeading(90)) {
            telemetry.update();
        }
        sleep(3000);
    }
}
