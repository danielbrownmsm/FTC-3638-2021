package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DriveToLineAuto", group = "")
public class DriveToLineAutoBetter extends LinearOpMode {
    private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(telemetry);
  
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode()  {
        drivetrain.init(hardwareMap); // init our robot
        waitForStart();

        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while(!drivetrain.driveDistance(82 - Constants.driftDistance)) { // this part up here drives
        }
    }
}