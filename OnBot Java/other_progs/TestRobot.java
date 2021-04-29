package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "TestRobot", group = "other")
public class TestRobot extends OpMode {
    private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(telemetry);
    private WobbleSubsystem wobble = new WobbleSubsystem(telemetry);
    private ShooterSubsystem shooter = new ShooterSubsystem(telemetry);
    
    
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        wobble.init(hardwareMap);
        shooter.init(hardwareMap);

        telemetry.addData("Status", "initialized, waiting for start");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //steps:
        //shooter:
            // verify trigger runs forwards
            // verify trigger runs backwards
            // verify shooter runs
            // verify shooter achieves correct velocity
        //wobble:
            // verify wobble arm moves left
            // right
            // up
            // verify wobble claw closes
            // verify wobble claw opens
        //intake:
            // verify intake moves up
            // verify intake moves down
            // verify intake moves neutral
        //drivetrain:
            // verify drivetrain runs forwards
            // backwards
            // left
            // right
            // turns
            // verify all of those have appropriate encoder values
            // verify gyro is calibrated
            // verify ring detection is working
        drivetrain.addTelemetry();
        telemetry.update();
        
        if (gamepad1.a) {
            drivetrain.resetEncoders();
            drivetrain.setHeading();
        }
    }
}