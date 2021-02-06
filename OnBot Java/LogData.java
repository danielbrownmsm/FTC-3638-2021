package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Log Data", group="Iterative Opmode")
public class LogData extends OpMode {
    private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(telemetry);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drivetrain.init(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("heading", drivetrain.getYaw());
        telemetry.addData("inches", drivetrain.getInches(drivetrain.getEncoderAverage()));
        drivetrain.postColorSensor();
        telemetry.update();
        
        if (gamepad1.a) {
            drivetrain.resetEncoders();
            drivetrain.setHeading();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}