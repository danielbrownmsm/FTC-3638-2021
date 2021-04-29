package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Const;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DoubleRandomWobbleAuto", group = "other")
@Disabled
public class DoubleRandomWobbleAuto extends LinearOpMode {
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

        wobble.setArm(Constants.wobbleServoUp);
        wobble.setClaw(Constants.wobbleClawClosed);
        waitForStart();
    
        /** Drive to the "B" target zone */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while(!drivetrain.driveDistance(105 - Constants.driftDistance)) { // does the actual driving
            telemetry.update();
        }
    
        /** Drop the wobble goal */
        wobble.setArm(Constants.wobbleServoLeft); // move the arm back over
        sleep(3000); // wait a sec for the servo
        wobble.setClaw(Constants.wobbleClawOpen); // open seasame
        sleep(1000); // same thing
        wobble.setArm(Constants.wobbleServoUp); // move the arm out of the way
        sleep(1000); // same thing
    
        /** Drive back to the other wobble goal */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while(!drivetrain.driveDistance(-84 + Constants.driftDistance)) { // does the actual driving
            telemetry.update();
        }

        /** Strafe to pick up the other wobble goal */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while (!drivetrain.strafeDistance(-15 + Constants.driftDistance)) {
            telemetry.update();
        }

        /** Pick up the other wobble goal */
        wobble.setArm(Constants.wobbleServoLeft);
        sleep(2000);
        wobble.setClaw(Constants.wobbleClawClosed); // close it
        sleep(2000);
        wobble.setArm(Constants.wobbleServoUp);
        sleep(3000); // lifting is hard
        
        /** Strafe to pick up the other wobble goal */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while (!drivetrain.strafeDistance(20 + Constants.driftDistance)) {
            telemetry.update();
        }

        /** Drive over to other drop zone or whatever */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while(!drivetrain.driveDistance(80 + Constants.driftDistance)) { // does the actual driving
            telemetry.update();
        }

        /** Spin */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while(!drivetrain.turnToHeading(180)) { // does the actual turning
            telemetry.update();
        }

        /** Drop the wobble goal */
        wobble.setArm(Constants.wobbleServoLeft); // move the arm back over
        sleep(3000); // wait a sec for the servo
        wobble.setClaw(Constants.wobbleClawOpen); // open seasame
        sleep(1000); // same thing
        wobble.setArm(Constants.wobbleServoUp); // move the arm out of the way
        sleep(1000); // same thing

        /** Drive back to the line */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while(!drivetrain.driveDistance(-15 + Constants.driftDistance)) { // does the actual driving
            telemetry.update();
        }
        sleep(1000); // and... stop
    }
}