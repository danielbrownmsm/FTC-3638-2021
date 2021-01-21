package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Const;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;

@TeleOp(name="AtomicTeleOp", group="Iterative Opmode")
public class AtomicTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        try {
            // driving (square inputs)
            float tempLeftStickX = Math.copySign(gamepad1.left_stick_x * gamepad1.left_stick_x, gamepad1.left_stick_x);
            float tempLeftStickY = Math.copySign(gamepad1.left_stick_y * gamepad1.left_stick_y, gamepad1.left_stick_y);
            float tempRightStickX = Math.copySign(gamepad1.right_stick_x * gamepad1.right_stick_x, gamepad1.right_stick_x);
            drivetrain.driveTeleOp(tempLeftStickX, tempLeftStickY, tempRightStickX);

            // wobble goal arm
            if (gamepad2.dpad_up) {
                wobble.setArm(Constants.wobbleServoUp);
            } else if (gamepad2.dpad_left) {
                wobble.setArm(Constants.wobbleServoLeft);
            } else if (gamepad2.dpad_right) {
                wobble.setArm(Constants.wobbleServoRight);
            }

            // wobble goal claw
            if (gamepad2.x) {
                wobble.setClaw(Constants.wobbleClawClosed);
            } else if (gamepad2.y) {
                wobble.setClaw(Constants.wobbleClawOpen);
            }

            /*if (gamepad2.left_bumper) {
                shooter.warmUp(1);
            } else if (gamepad2.right_bumper) {
                shooter.shoot(1);
            } else if (gamepad2.b) {
                shooter.shoot(0);
            }*/
            
            shooter.setTrigger(gamepad2.right_trigger);
            if (gamepad2.a) {
                shooter.warmUp(1);
            } else {
                shooter.warmUp(0);
            }

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