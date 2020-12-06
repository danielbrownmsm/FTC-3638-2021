package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RandomWobbleAuto", group = "")
public class RandomWobbleAuto extends LinearOpMode {
  private CustomRobot robot = new CustomRobot();
  
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode()  {
    robot.init(hardwareMap); // init our robot (auto sets the wobble claw to closed)
    robot.setRingArmTarget(30); // so the arm doesn't go kaboom
    robot.setWobbleArm(Constants.wobbleServoRight); // so we hold up the wobble arm
    waitForStart();
    
    /** Drive to the "B" target zone */
    robot.resetEncoders(); // prepare ourselves
    robot.setHeading(); // to drive
    robot.setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // so we stop when we want to
    while(!robot.driveDistance(120 - Constants.driftDistance)) { // does the actual driving
      robot.periodic(); // makes driving work and arm not blow up
    }
    
    /** Drop the wobble goal */
    robot.driveTeleOp(0, 0, 0); // stop, just to be sure
    robot.setWobbleArm(Constants.wobbleServoLeft); // move the arm back over
    sleep(3000); // wait a sec for the servo
    robot.setWobbleClaw(true); // open seasame
    sleep(1000); // same thing
    
    /** Drive back to the line */
    robot.resetEncoders(); // prepare ourselves
    robot.setHeading(); // to drive
    robot.setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // so we stop when we want to
    while(!robot.driveDistance(-60 + Constants.driftDistance)) { // does the actual driving
      robot.periodic(); // makes driving work and arm not blow up
    }
    sleep(10); //?
    robot.driveTeleOp(0, 0, 0); // hard-stop
    sleep(1000); // just to be safe
  }
}
