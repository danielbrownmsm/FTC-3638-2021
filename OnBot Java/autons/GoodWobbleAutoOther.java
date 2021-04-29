package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Const;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "GoodWobbleAuto Red", group = "", preselectTeleOp = "AtomicTeleOp2")
public class GoodWobbleAutoOther extends LinearOpMode {
    private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(telemetry);
    private WobbleSubsystem wobble = new WobbleSubsystem(telemetry);
    private ShooterSubsystem shooter = new ShooterSubsystem(telemetry);
    private double inchesToDrive = 0;
    private double path = -1;
  
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
        
        shooter.setIntake(Constants.intakeAuto);
        wobble.setArm(Constants.wobbleServoUp);
    
        /** Drive to the rings */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while(!drivetrain.driveDistance(22 - Constants.driftDistance) && opModeIsActive()) { // does the actual driving
            drivetrain.addTelemetry();
            wobble.addTelemetry();
            shooter.addTelemetry();
            
            telemetry.update();
        }
        
        // turn to make sure we are lined up
        //while(!drivetrain.turnToHeading(drivetrain.getLastSetHeading())) {
        //    drivetrain.postColorSensor();
        //    telemetry.update();
        //}
        
        sleep(1000); // wait a bit to get a steady reading
        if (drivetrain.getRingCount() == 4) { // NaN not equal to itself
            inchesToDrive = 123 - 22; // -24 b/c we've already driven that much to get to the rings
            path = 3;
        } else if (drivetrain.getRingCount() == 1) {
            inchesToDrive = 87 - 22;
            path = 2;
        } else if (drivetrain.getRingCount() == 0) {
            inchesToDrive = 75 - 22;
            path = 1;
        }
        
        /** Strafe because otherwise we run directly into the stack */
        drivetrain.resetEncoders(); // prepare ourselves
        while (!drivetrain.strafeDistance(-11 + Constants.driftDistance) && opModeIsActive()) {
            drivetrain.addTelemetry();
            wobble.addTelemetry();
            shooter.addTelemetry();
            
            telemetry.update();
        }
        
        /** Drive to the target zone */
        drivetrain.resetEncoders(); // prepare ourselves
        while(!drivetrain.driveDistance(inchesToDrive - Constants.driftDistance) && opModeIsActive()) {
            drivetrain.addTelemetry();
            wobble.addTelemetry();
            shooter.addTelemetry();
            
            telemetry.update();
        }
        
        if (path == 1) {
            // turn so wobble goal mech is on correct side
            while(!drivetrain.turnToHeading(178)) {
                drivetrain.addTelemetry();
                wobble.addTelemetry();
                shooter.addTelemetry();
            
                telemetry.update();
            }
            
            // strafe right (but actually left b/c we turned)
            drivetrain.resetEncoders(); // prepare ourselves
            while (!drivetrain.strafeDistance(-30 + Constants.driftDistance) && opModeIsActive()) {
                drivetrain.addTelemetry();
                wobble.addTelemetry();
                shooter.addTelemetry();
            
                telemetry.update();
            }
        } else if (path == 2) {
            //strafe a bit to make sure we're completely in
            drivetrain.resetEncoders(); // prepare ourselves
            while (!drivetrain.strafeDistance(35 + Constants.driftDistance) && opModeIsActive()) {
                drivetrain.addTelemetry();
                wobble.addTelemetry();
                shooter.addTelemetry();
            
                telemetry.update();
            }
        } else if (path == 3) {
            // turn
            while(!drivetrain.turnToHeading(175) && opModeIsActive()) {
                drivetrain.addTelemetry();
                wobble.addTelemetry();
                shooter.addTelemetry();
            
                telemetry.update();
            }
            
            // strafe
            drivetrain.resetEncoders(); // prepare ourselves
            while (!drivetrain.strafeDistance(-37 + Constants.driftDistance) && opModeIsActive()) {
                drivetrain.addTelemetry();
                wobble.addTelemetry();
                shooter.addTelemetry();
            
                telemetry.update();
            }
        }
        
        /** Drop the wobble goal */
        wobble.setArm(Constants.wobbleServoLeft); // move the arm back over
        sleep(2000); // wait a sec for the servo
        wobble.setClaw(Constants.wobbleClawOpen); // open seasame
        sleep(1000); // same thing
        wobble.setArm(Constants.wobbleServoUp); // move the arm out of the way
        sleep(1000); // same thing
        
        if (path == 3) {
            /** Strafe so we're not touching wobble goal */
            drivetrain.resetEncoders(); // prepare ourselves
            while (!drivetrain.strafeDistance(4 + Constants.driftDistance) && opModeIsActive()) {
                drivetrain.addTelemetry();
                wobble.addTelemetry();
                shooter.addTelemetry();
                
                telemetry.update();
            }
            
            inchesToDrive = 42;
        } else if (path == 2) {
            /** Strafe so we're not touching wobble goal */
            drivetrain.resetEncoders(); // prepare ourselves
            while (!drivetrain.strafeDistance(4 + Constants.driftDistance) && opModeIsActive()) {
                drivetrain.addTelemetry();
                wobble.addTelemetry();
                shooter.addTelemetry();
                
                telemetry.update();
            }
            
            inchesToDrive = -18;
        } else {
            inchesToDrive = 1; // we should be close
        }
        
        /** Drive to the target zone */
        drivetrain.resetEncoders(); // prepare ourselves
        drivetrain.setHeading(); // to drive
        while(!drivetrain.driveDistance(inchesToDrive - Constants.driftDistance) && opModeIsActive()) { // does the actual driving
            drivetrain.addTelemetry();
            wobble.addTelemetry();
            shooter.addTelemetry();
            
            telemetry.update();
        }
        sleep(1000);
        
        /** Strafe so we're not touching wobble goal */
        drivetrain.resetEncoders(); // prepare ourselves
        while (!drivetrain.strafeDistance(7 + Constants.driftDistance) && opModeIsActive()) {
            drivetrain.addTelemetry();
            wobble.addTelemetry();
            shooter.addTelemetry();
            
            telemetry.update();
        }
        sleep(1000);
    }
}