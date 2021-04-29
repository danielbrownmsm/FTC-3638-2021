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
    private double servo1Pos = 0.7;
    private double servo2Pos = 0;
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
            drivetrain.driveTeleOp(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x); // normal inputs
 
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
                if (servo1Pos > 0.7) {
                    servo1Pos -= 0.05;
                }
            } else if (gamepad1.left_bumper) {
                shooter.setIntake(Constants.intakeUp); // up is 0
                if (servo1Pos < 0.95) {
                    servo1Pos += 0.05;
                }
            }

            // shooter stuff
            if (gamepad1.a) {
                shooter.shoot(Constants.targetRPM);
                shooter.setAngle(servo1Pos);
            } else {
                shooter.shoot(0);
            }

            shooter.setTrigger(gamepad1.right_trigger - gamepad1.left_trigger);

            drivetrain.addTelemetry();
            shooter.addTelemetry();
            wobble.addTelemetry();
            
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            telemetry.addData("Servo 1 Pos", servo1Pos);
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