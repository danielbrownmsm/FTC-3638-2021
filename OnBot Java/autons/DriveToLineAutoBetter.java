package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "DriveToLineAuto", group = "other")
public class DriveToLineAutoBetter extends LinearOpMode {
    private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(telemetry);
    private WobbleSubsystem wobble = new WobbleSubsystem(telemetry);
    private ShooterSubsystem shooter = new ShooterSubsystem(telemetry);
    
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode()  {
        drivetrain.init(hardwareMap);
        wobble.init(hardwareMap);
        shooter.init(hardwareMap);

        wobble.setArm(Constants.wobbleServoRight);
        wobble.setClaw(Constants.wobbleClawClosed);
        shooter.setIntake(Constants.intakeAuto);
        waitForStart();
        
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        /*while(!drivetrain.driveDistance(69 - Constants.driftDistance) && opModeIsActive()) { // this part up here drives
            drivetrain.addTelemetry();
            telemetry.update();
        }*/
        //sleep(1000);
        
        while(!drivetrain.turnToHeading(180) && opModeIsActive()) {
            drivetrain.addTelemetry();
            telemetry.update();
        }
        
        sleep(1000);
    }
}