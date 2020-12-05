package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.CustomRobot;

@TeleOp(name="TeleOp", group="Iterative Opmode")
public class AtomicTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private long lastTime = 0;
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
        robot.setRingClaw(true);
        robot.setWobbleArm(Constants.wobbleServoUp);
        robot.incrementRingArm(30);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        lastTime = runtime.nanoseconds();
        // driving (square inputs)
        float tempLeftStickX = Math.copySign(gamepad1.left_stick_x * gamepad1.left_stick_x, gamepad1.left_stick_x);
        float tempLeftStickY = Math.copySign(gamepad1.left_stick_y * gamepad1.left_stick_y, gamepad1.left_stick_y);
        float tempRightStickX = Math.copySign(gamepad1.right_stick_x * gamepad1.right_stick_x, gamepad1.right_stick_x);
        robot.driveTeleOp(tempLeftStickX, tempLeftStickY, tempRightStickX);
        
        telemetry.addData("Driving ns: ", lastTime - runtime.nanoseconds());
        lastTime = runtime.nanoseconds();

        // ring arm
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            robot.incrementRingArm(1);
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            robot.incrementRingArm(-1);
        }
        telemetry.addData("Arm Target: ", robot.ringArmTargetPosition); //yes, I made a public variable. Deal with it.
        telemetry.addData("Ring Arm ns: ", lastTime - runtime.nanoseconds());
        lastTime = runtime.nanoseconds();
        
        // ring claw
        if (gamepad1.y || gamepad2.y) {
            robot.setRingClaw(true);
        } else if (gamepad1.x || gamepad2.x) {
            robot.setRingClaw(false);
        } else if (gamepad1.back || gamepad2.back) {
            robot.ringClawKick();
        }
        telemetry.addData("Ring Claw ns: ", lastTime - runtime.nanoseconds());
        lastTime = runtime.nanoseconds();
        
        // wobble goal arm
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            robot.setWobbleArm(Constants.wobbleServoUp);
        } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
            robot.setWobbleArm(Constants.wobbleServoLeft);
        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            robot.setWobbleArm(Constants.wobbleServoRight);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            robot.setWobbleArm(Constants.wobbleServoDown);
        }
        telemetry.addData("Wobble Arm ns: ", lastTime - runtime.nanoseconds());
        lastTime = runtime.nanoseconds();
        
        // wobble goal claw
        if (gamepad1.a || gamepad2.a) {
            robot.setWobbleClaw(true);
        } else if (gamepad1.b || gamepad2.b) {
            robot.setWobbleClaw(false);
        }
        telemetry.addData("Wobble Claw ns: ", lastTime - runtime.nanoseconds());
        lastTime = runtime.nanoseconds();
        
        // finally
        robot.periodic();

        telemetry.addData("Periodic ns: ", lastTime - runtime.nanoseconds());
        lastTime = runtime.nanoseconds();
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //robot.stop();
    }

}