package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.CustomRobot;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;

@TeleOp(name="AtomicTeleOp", group="Iterative Opmode")
public class AtomicTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private CustomRobot robot = new CustomRobot();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.setWobbleArm(Constants.wobbleServoUp);
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
            robot.driveTeleOp(tempLeftStickX, tempLeftStickY, tempRightStickX);

            // wobble goal arm
            if (gamepad2.dpad_up) {
                robot.setWobbleArm(Constants.wobbleServoUp);
            } else if (gamepad2.dpad_left) {
                robot.setWobbleArm(Constants.wobbleServoLeft);
            } else if (gamepad2.dpad_right) {
                robot.setWobbleArm(Constants.wobbleServoRight);
            }

            // wobble goal claw
            if (gamepad2.a) {
                robot.setWobbleClaw(true);
            } else if (gamepad2.b) {
                robot.setWobbleClaw(false);
            }

        } catch (TargetPositionNotSetException targetPosError) {
            telemetry.addData("The target position error happened again", "");
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //robot.stop();
    }

}