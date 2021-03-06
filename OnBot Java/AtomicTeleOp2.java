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

@TeleOp(name="AtomicTeleOp2", group="Iterative Opmode")
public class AtomicTeleOp2 extends OpMode
{
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
        drivetrain.postImuStatus();
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
            // driving (square inputs)
            //float tempLeftStickX = Math.copySign(gamepad1.left_stick_x * gamepad1.left_stick_x, gamepad1.left_stick_x);
            //float tempLeftStickY = Math.copySign(gamepad1.left_stick_y * gamepad1.left_stick_y, gamepad1.left_stick_y);
            //float tempRightStickX = Math.copySign(gamepad1.right_stick_x * gamepad1.right_stick_x, gamepad1.right_stick_x);
            //drivetrain.driveTeleOp(tempLeftStickX, tempLeftStickY, tempRightStickX);
            if (!halfSpeed) { // if the stick is pressed down
                drivetrain.driveTeleOp(gamepad1.left_stick_x * direction, gamepad1.left_stick_y * direction, gamepad1.right_stick_x); // normal inputs
            } else { // halve the inputs
                drivetrain.driveTeleOp(gamepad1.left_stick_x * direction / 2, gamepad1.left_stick_y  * direction / 2, gamepad1.right_stick_x / 2);
            }
            
            if (gamepad1.back && !wasToggled) {
                direction *= -1;
                wasToggled = true;
            } else if (!gamepad1.back) {
                wasToggled = false;
            }

            //if ((gamepad1.left_stick_button || gamepad1.right_stick_button) && !wasHalfSpeedToggled) {
            //    wasHalfSpeedToggled = true;
            //    halfSpeed = !halfSpeed;
            //} else if (!gamepad1.left_stick_button || !gamepad1.right_stick_button) {
            //    wasHalfSpeedToggled = false;
            //}

            // wobble goal arm
            if (gamepad2.dpad_up) {
                wobble.setArm(Constants.wobbleServoUp);
            } else if (gamepad2.dpad_left) {
                wobble.setArm(Constants.wobbleServoLeft);
            } else if (gamepad2.dpad_right) {
                wobble.setArm(Constants.wobbleServoRight);
            } else if (gamepad2.dpad_down) {
                wobble.setArm(Constants.wobbleServoDown);
            }

            // wobble goal claw
            if (gamepad2.x) {
                wobble.setClaw(Constants.wobbleClawClosed);
            } else if (gamepad2.y) {
                wobble.setClaw(Constants.wobbleClawOpen);
            }
            
            if (gamepad1.b) {
                shooter.setIntake(Constants.intakeNeutral); // net is 0.2
            } else if (gamepad1.right_bumper) {
                shooter.setIntake(Constants.intakeDown); // down is 0.7
            } else if (gamepad1.left_bumper) {
                shooter.setIntake(Constants.intakeUp); // up is 0
            }

            /*if (gamepad2.left_bumper) {
                shooter.warmUp(1);
            } else if (gamepad2.right_bumper) {
                shooter.shoot(1);
            } else if (gamepad2.b) {
                shooter.shoot(0);
            }*/
            
            if (gamepad2.a || gamepad1.a) {
                shooter.shootPID(runtime.seconds());
            } else {
                shooter.warmUp(0);
            }

            //if (shooter.isShooterGood()) {
                shooter.setTrigger(gamepad1.right_trigger - gamepad1.left_trigger + gamepad2.right_trigger - gamepad2.left_trigger);
            //}

            telemetry.addData("Shooter Velocity: ", shooter.getVelocity(runtime.seconds()));
            //telemetry.addData("Time", runtime.seconds());
            telemetry.update();

        } catch (TargetPositionNotSetException targetPosError) {
            telemetry.addData("The target position error happened again", "");
            telemetry.update();
        } catch (Exception exception) {
            telemetry.addData("Exception occured", exception.toString());
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