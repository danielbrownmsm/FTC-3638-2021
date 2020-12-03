package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.CustomRobot;

@TeleOp(name="TeleOp", group="Iterative Opmode")
public class AtomicTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // driving
        float tempLeftStickX = Math.copySign(gamepad1.left_stick_x * gamepad1.left_stick_x, gamepad1.left_stick_x);
        float tempLeftStickY = Math.copySign(gamepad1.left_stick_y * gamepad1.left_stick_y, gamepad1.left_stick_y);
        float tempRightStickX = Math.copySign(gamepad1.right_stick_x * gamepad1.right_stick_x, gamepad1.right_stick_x);
        robot.driveTeleOp(tempLeftStickX, tempLeftStickY, tempRightStickX);

        // ring arm
        if (gamepad1.right_bumper) {
            robot.incrementRingArm(1);
        } else if (gamepad1.left_bumper) {
            robot.incrementRingArm(-1);
        }
        
        // ring claw
        if (gamepad1.y) {
            robot.setRingClaw(true);
        } else if (gamepad1.x) {
            robot.setRingClaw(false);
        } else if (gamepad1.back) {
            robot.ringClawKick();
        }
        
        // wobble goal arm
        if (gamepad1.dpad_up) {
            robot.setWobbleArm(Constants.wobbleServoUp);
        } else if (gamepad1.dpad_left) {
            robot.setWobbleArm(Constants.wobbleServoLeft);
        } else if (gamepad1.dpad_right) {
            robot.setWobbleArm(Constants.wobbleServoRight);
        } else if (gamepad1.dpad_down) {
            robot.setWobbleArm(Constants.wobbleServoDown);
        }
        
        // wobble goal claw
        if (gamepad1.a) {
            robot.setWobbleClaw(true);
        } else if (gamepad1.b) {
            robot.setWobbleClaw(false);
        }
        
        // finally
        robot.periodic();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //robot.stop();
    }

}