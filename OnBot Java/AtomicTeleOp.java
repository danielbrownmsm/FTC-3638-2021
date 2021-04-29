package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Const;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;

@TeleOp(name="AtomicTeleOp", group="Iterative Opmode")
public class AtomicTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(telemetry);
    private WobbleSubsystem wobble = new WobbleSubsystem(telemetry);
    private ShooterSubsystem shooter = new ShooterSubsystem(telemetry);
    
    private int direction = 1;
    private boolean wasToggled = false;
    private boolean wasHalfSpeedToggled = false;
    private boolean halfSpeed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        wobble.init(hardwareMap);
        shooter.init(hardwareMap);
        
        drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        drivetrain.addTelemetry();
        shooter.addTelemetry();
        wobble.addTelemetry();
        
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        runtime.startTime();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        try {
            //if (!halfSpeed) {
                drivetrain.driveTeleOp(gamepad1.left_stick_x * direction, gamepad1.left_stick_y * direction, gamepad1.right_stick_x); // normal inputs
            //} else { // halve the inputs
            //    drivetrain.driveTeleOp(gamepad1.left_stick_x * direction / 2, gamepad1.left_stick_y  * direction / 2, gamepad1.right_stick_x / 2);
            //}
            
            // reverse-direction stuff
            /*
            if (gamepad1.back && !wasToggled) {
                direction *= -1;
                wasToggled = true;
            } else if (!gamepad1.back) {
                wasToggled = false;
            }
            */

            // wobble goal arm
            if (gamepad1.dpad_up) {
                wobble.setArm(Constants.wobbleServoUp);
            } else if (gamepad1.dpad_left) {
                wobble.setArm(Constants.wobbleServoLeft);
            } else if (gamepad1.dpad_right) {
                wobble.setArm(Constants.wobbleServoRight);
            }

            // wobble goal claw
            if (gamepad1.x) {
                wobble.setClaw(Constants.wobbleClawClosed);
            } else if (gamepad1.y) {
                wobble.setClaw(Constants.wobbleClawOpen);
            }
            
            // intake stuff
            if (gamepad1.b) {
                shooter.setIntake(Constants.intakeNeutral); // net is 0.2
            } else if (gamepad1.right_bumper) {
                shooter.setIntake(Constants.intakeDown); // down is 0.7
            } else if (gamepad1.left_bumper) {
                shooter.setIntake(Constants.intakeUp); // up is 0
            }

            // shooter stuff
            if (gamepad1.a) {
                shooter.shootPID(runtime.seconds());
            } else {
                shooter.warmUp(0);
            }
            
            // move this to top b/c it's drivetrain stuff
            if (gamepad1.start) {
                drivetrain.resetEncoders();
                drivetrain.setHeading();
                while (!drivetrain.strafeDistance(-16) && !gamepad1.back) {
                    drivetrain.addTelemetry();
                    shooter.addTelemetry();
                    wobble.addTelemetry();
                    
                    telemetry.update();
                }
                drivetrain.setHeading();
                while (!drivetrain.turnToHeading(-30) && !gamepad1.back) {
                    drivetrain.addTelemetry();
                    shooter.addTelemetry();
                    wobble.addTelemetry();
                    
                    telemetry.update();
                }
            }

            shooter.setTrigger(gamepad1.right_trigger - gamepad1.left_trigger);

            drivetrain.addTelemetry();
            shooter.addTelemetry();
            wobble.addTelemetry();
            
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            telemetry.addData("Time", runtime.seconds());
            
            telemetry.update();

        } catch (TargetPositionNotSetException targetPosError) {
            telemetry.addData("The target position error happened again", "");
            telemetry.update();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}