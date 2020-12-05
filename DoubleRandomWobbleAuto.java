package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DoubleRandomWobbleAuto", group = "")
public class DoubleRandomWobbleAuto extends LinearOpMode {
  private CustomRobot robot = new CustomRobot();
  
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode()  {
    robot.init(hardwareMap); // init our robot (auto sets the wobble claw to closed)
    robot.setRingArmTarget(30); // so the arm doesn't go kaboom
    robot.setWobbleArm(Constants.wobbleServoRight); // so we hold up the wobble arm
    robot.setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // so we stop when we want to
    waitForStart();
    
    /** Drive to the "B" target zone */
    robot.resetEncoders(); // prepare ourselves
    robot.setHeading(); // to drive
    while(!robot.driveDistance(120 - Constants.driftDistance)) { // does the actual driving
      robot.periodic(); // makes driving work and arm not blow up
    }
    robot.driveTeleOp(0, 0, 0); // stop, just to be sure
    
    /** Drop the wobble goal */
    robot.setWobbleArm(Constants.wobbleServoLeft); // move the arm back over
    sleep(3000); // wait a sec for the servo
    robot.setWobbleClaw(true); // open seasame
    sleep(1000); // same thing
    robot.setWobbleArm(Constants.wobbleServoUp); // move the arm out of the way
    sleep(1000); // same thing
    
    /** Drive back to the other wobble goal */
    robot.resetEncoders(); // prepare ourselves
    robot.setHeading(); // to drive
    while(!robot.driveDistance(-100 + Constants.driftDistance)) { // does the actual driving
      robot.periodic(); // makes driving work and arm not blow up
    }
    robot.driveTeleOp(0, 0, 0); // hard-stop

    /** Strafe to pick up the other wobble goal */
    robot.resetEncoders(); // prepare ourselves
    robot.setHeading(); // to drive
    while (!robot.strafeDistance(20 + Constants.driftDistance)) {
      robot.periodic();
    }
    robot.driveTeleOp(0, 0, 0);

    /** Pick up the other wobble goal */
    robot.setWobbleArm(Constants.wobbleServoLeft);
    sleep(2000);
    robot.setWobbleClaw(false); // close it
    sleep(2000);
    robot.setWobbleArm(Constants.wobbleServoUp);
    sleep(3000); // lifting is hard
    
    /** Drive over to other drop zone or whatever */
    robot.resetEncoders(); // prepare ourselves
    robot.setHeading(); // to drive
    while(!robot.driveDistance(80 + Constants.driftDistance)) { // does the actual driving
      robot.periodic(); // makes driving work and arm not blow up
    }
    robot.driveTeleOp(0, 0, 0); // hard-stop

    /** Spin */
    robot.resetEncoders(); // prepare ourselves
    robot.setHeading(); // to drive
    while(!robot.turnToHeading(180)) { // does the actual turning
      robot.periodic(); // makes driving work and arm not blow up
    }
    robot.driveTeleOp(0, 0, 0); // hard-stop
    
    /** Drop the wobble goal */
    robot.setWobbleArm(Constants.wobbleServoLeft); // move the arm back over
    sleep(3000); // wait a sec for the servo
    robot.setWobbleClaw(true); // open seasame
    sleep(1000); // same thing
    robot.setWobbleArm(Constants.wobbleServoUp); // move the arm out of the way
    sleep(1000); // same thing

    /** Drive back to the line */
    robot.resetEncoders(); // prepare ourselves
    robot.setHeading(); // to drive
    while(!robot.driveDistance(-30 + Constants.driftDistance)) { // does the actual driving
      robot.periodic(); // makes driving work and arm not blow up
    }
    robot.driveTeleOp(0, 0, 0); // hard-stop
    sleep(1000); // and... stop
  }
}
