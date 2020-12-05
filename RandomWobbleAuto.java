package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Const;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;

@Autonomous(name = "RandomWobbleAuto", group = "")
public class RandomWobbleAuto extends LinearOpMode {
  private CustomRobot robot = new CustomRobot(); //?
  
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode()  {
    robot.init(hardwareMap);
    robot.setRingArmTarget(30);
    robot.setWobbleArm(Constants.wobbleServoRight);
    waitForStart();
    
    robot.resetEncoders();
    robot.setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    while(!robot.driveDistance(120 - Constants.driftDistance)) {
      robot.periodic();
    }
    robot.driveTeleOp(0, 0, 0);
    //robot.turnToHeading(180);
    robot.setWobbleArm(Constants.wobbleServoLeft);
    sleep(2000); // wait a sec for the servo
    robot.setWobbleClaw(true);
    sleep(2000); // same thing
  }
}
