package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TestRobotAuto", group = "")
public class TestRobotAuto extends LinearOpMode {
  private CustomRobot robot = new CustomRobot();
  
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode()  {
    telemetry.addData("Status", "Initializing...");
    telemetry.update();

    robot.init(hardwareMap); // init our robot
    telemetry.addData("Status", "Waiting for start...");
    telemetry.update();

    waitForStart();

    telemetry.addData("Status", "Testing...");
    telemetry.update();
    robot.testRobot();
    
    sleep(1000); // maybe so function doesn't return immediately?
    telemetry.addData("Status", "Done! All tests passed!");
    telemetry.update();
  }
}
