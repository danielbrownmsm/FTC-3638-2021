package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DriveToLineAuto", group = "")
public class DriveToLineAutoBetter extends LinearOpMode {
  private CustomRobot robot = new CustomRobot(); //?
  
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode()  {
    robot.init(hardwareMap); // init our robot
    robot.setRingArmTarget(30); // get the arm ready to raise itself so it doesn't get in the way
    waitForStart();
    
    robot.resetEncoders(); // prepare ourselves
    robot.setHeading(); // to drive
    robot.setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // so we don't drift after we stop
    while(!robot.driveDistance(82 - Constants.driftDistance)) { // this part up here drives
      robot.periodic(); // this part here makes this loop last the entire time without the opmode freakin stopping immediately
      // because I guess functions are non-blocking. Also makes sure the arm doesn't go kaboom
    }
    robot.driveTeleOp(0, 0, 0);
  }
}
