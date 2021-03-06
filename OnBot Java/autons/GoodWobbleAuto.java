package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Const;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "GoodWobbleAuto", group = "")
public class GoodWobbleAuto extends LinearOpMode {
    private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(telemetry);
    private WobbleSubsystem wobble = new WobbleSubsystem(telemetry);
    private ShooterSubsystem shooter = new ShooterSubsystem(telemetry);
    private double inchesToDrive = 0;
  
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode()  {
        drivetrain.init(hardwareMap);
        wobble.init(hardwareMap);
        shooter.init(hardwareMap);

        wobble.setArm(Constants.wobbleServoDown);
        wobble.setClaw(Constants.wobbleClawClosed);
        waitForStart();
    
        /** Drive to the rings */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while(!drivetrain.driveDistance(22 - Constants.driftDistance)) { // does the actual driving
            drivetrain.postColorSensor();
            telemetry.update();
        }
        
        sleep(1000); // wait a bit to get a steady reading
        if (drivetrain.getRingCount() == 4) { // target zone C
            inchesToDrive = 123 - 22; // -24 b/c we've already driven that much to get to the rings
        } else { // target zone A, assumes no rings
            inchesToDrive = 74 - 22;
        }
        
        /** Strafe because otherwise we run directly into the stack */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while (!drivetrain.strafeDistance(-11 + Constants.driftDistance)) {
            telemetry.update();
        }
        
        /** Drive to the target zone */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while(!drivetrain.driveDistance(inchesToDrive - Constants.driftDistance)) { // does the actual driving
            telemetry.update();
        }
        
        /** Drop the wobble goal */
        wobble.setArm(Constants.wobbleServoLeft); // move the arm back over
        sleep(3000); // wait a sec for the servo
        wobble.setClaw(Constants.wobbleClawOpen); // open seasame
        sleep(1000); // same thing
        wobble.setArm(Constants.wobbleServoUp); // move the arm out of the way
        sleep(1000); // same thing
        
        if (inchesToDrive > 82) {
            inchesToDrive = -36; // the answer to Life, the Universe, and Everything
        } else {
            inchesToDrive = -1; // we should be close
        }
        
        /** Drive to the target zone */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while(!drivetrain.driveDistance(inchesToDrive - Constants.driftDistance)) { // does the actual driving
            telemetry.update();
        }
        
        /** Strafe so we're not touching wobble goal */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while (!drivetrain.strafeDistance(10 + Constants.driftDistance)) {
            telemetry.update();
        }
        sleep(1000);
    }
}
