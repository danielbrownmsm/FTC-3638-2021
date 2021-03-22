/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/*
 * <p>"This device requires calibration in order to operate accurately. [...] Calibration data is
 * lost on a power cycle. See one of the examples for a description of how to calibrate the device,
 * but in essence:</p>
 *
 *
 * <li>
 *     <ol>GYR: Simply let the sensor sit flat for a few seconds.</ol>
 *     <ol>ACC: Move the sensor in various positions. Start flat, then rotate slowly by 45
 *              degrees, hold for a few seconds, then continue rotating another 45 degrees and
 *              hold, etc. 6 or more movements of this type may be required. You can move through
 *              any axis you desire, but make sure that the device is lying at least once
 *              perpendicular to the x, y, and z axis.</ol>
 *     <ol>MAG: Move slowly in a figure 8 pattern in the air, until the calibration values reaches 3.</ol>
 *     <ol>SYS: This will usually reach 3 when the other items have also reached 3. If not, continue
 *              slowly moving the device though various axes until it does."</ol>
 * </li>
 *
 * <p>To calibrate the IMU, run this sample opmode with a gamepad attached to the driver station.
 * Once the IMU has reached sufficient calibration as reported on telemetry, press the 'A'
 * button on the gamepad to write the calibration to a file. That file can then be indicated
 * later when running an opmode which uses the IMU.</p>
 *
 */

@TeleOp(name="Calibrate Gyro", group="other")
public class CalibrateGyro extends LinearOpMode {
    private BNO055IMU imu1;
    private BNO055IMU imu2;

    @Override public void runOpMode() {
        telemetry.log().setCapacity(12);
        telemetry.log().add("Press 'a' to write imu 1 calib data, 'b' for imu 2");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu1.initialize(parameters);

        imu2 = hardwareMap.get(BNO055IMU.class, "imu 2");
        imu2.initialize(parameters);

        telemetry.log().add("Waiting for start...");

        // Wait until we're told to go
        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        while (opModeIsActive()) {

            if (gamepad1.a) {

                // Get the calibration data
                BNO055IMU.CalibrationData calibrationData = imu1.readCalibrationData();

                String filename = "imu1_calib.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                telemetry.log().add("saved to '%s'", filename);

                // Wait for the button to be released
                while (gamepad1.a) {
                    telemetry.update();
                    idle();
                }
            }
            
            if (gamepad1.b) {

                // Get the calibration data
                BNO055IMU.CalibrationData calibrationData = imu2.readCalibrationData();

                String filename = "imu2_calib.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                telemetry.log().add("saved to '%s'", filename);

                // Wait for the button to be released
                while (gamepad1.b) {
                    telemetry.update();
                    idle();
                }
            }
            
            telemetry.addData("IMU 1", imu1.getCalibrationStatus().toString());
            telemetry.addData("IMU 2", imu2.getCalibrationStatus().toString());

            telemetry.update();
        }
    }
}