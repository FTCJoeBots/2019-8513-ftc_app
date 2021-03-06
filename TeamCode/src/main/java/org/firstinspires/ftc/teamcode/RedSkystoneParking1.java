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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "Red Skystone Parking Bridge", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class RedSkystoneParking1 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot2019 robot = new HardwareJoeBot2019();
    Robot8513 utility = new Robot8513();

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has an IR proximity sensor which is used to calculate distance and an RGB color sensor.
     * 
     * There will be some variation in the values measured depending on whether you are using a
     * V3 color sensor versus the older V2 and V1 sensors, as the V3 is based around a different chip.
     *
     * For V1/V2, the light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * For V3, the distance sensor as configured can handle distances between 0.25" (~0.6cm) and 6" (~15cm).
     * Any target closer than 0.25" will dislay as 0.25" and any target farther than 6" will display as 6".
     *
     * Note that the distance sensor function of both chips is built around an IR proximity sensor, which is
     * sensitive to ambient light and the reflectivity of the surface against which you are measuring. If
     * very accurate distance is required you should consider calibrating the raw optical values read from the
     * chip to your exact situation.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.
     *
     */

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);
        utility.init(hardwareMap, this);

        // get a reference to the color sensor.
        utility.colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        utility.colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");

        // get a reference to the distance sensor that shares the same name.
        utility.distanceSensorRight = hardwareMap.get(DistanceSensor.class, "colorSensorRight");
        utility.distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "colorSensorLeft");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValuesRight[] = {0F, 0F, 0F};
        float hsvValuesLeft[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float valuesRight[] = hsvValuesRight;
        final float valuesLeft[] = hsvValuesLeft;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        double SkystonePos = 50;
        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.

            robot.moveInches(21, 0.25, 5);
            sleep(600);
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (utility.colorSensorRight.red() * SCALE_FACTOR),
                    (int) (utility.colorSensorRight.green() * SCALE_FACTOR),
                    (int) (utility.colorSensorRight.blue() * SCALE_FACTOR),
                    hsvValuesRight);

            Color.RGBToHSV((int) (utility.colorSensorLeft.red() * SCALE_FACTOR),
                    (int) (utility.colorSensorLeft.green() * SCALE_FACTOR),
                    (int) (utility.colorSensorLeft.blue() * SCALE_FACTOR),
                    hsvValuesLeft);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm) Right",
                    String.format(Locale.US, "%.02f", utility.distanceSensorRight.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha Right", utility.colorSensorRight.alpha());
            telemetry.addData("Red Right", utility.colorSensorRight.red());
            telemetry.addData("Green Right", utility.colorSensorRight.green());
            telemetry.addData("Blue Right", utility.colorSensorRight.blue());
            telemetry.addData("Hue Right", hsvValuesRight[0]);

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", utility.distanceSensorLeft.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", utility.colorSensorLeft.alpha());
            telemetry.addData("Red  ", utility.colorSensorLeft.red());
            telemetry.addData("Green", utility.colorSensorLeft.green());
            telemetry.addData("Blue ", utility.colorSensorLeft.blue());
            telemetry.addData("Hue", hsvValuesLeft[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, valuesRight));
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, valuesLeft));

                }
            });


            telemetry.update();

            //SkystonePos = utility.checkSkystone(hsvValuesRight[0], hsvValuesLeft[0]);

            if (utility.checkSkystoneRed(hsvValuesRight[0], hsvValuesLeft[0]) == 1) {

                robot.strafeSeconds(640, 0.2);
                SkystonePos = SkystonePos - 8;

            }

            if (utility.checkSkystoneRed(hsvValuesRight[0], hsvValuesLeft[0]) == 2) {

                sleep(650);
            }

            if (utility.checkSkystoneRed(hsvValuesRight[0], hsvValuesLeft[0]) == 3) {

                robot.strafeSeconds(640, -0.2);
                SkystonePos = SkystonePos + 4;
            }

            sleep(1000);
            robot.moveInches(8.5, 0.3, 5);
            sleep(300);
            utility.closeClamp();
            sleep(500);
            robot.moveInches(-8, .5, 7);
            sleep(300);
            robot.rotateDegrees(80, 0.4);
            sleep(300);
            robot.moveInches(SkystonePos, 0.4, 5);
            sleep(300);
            utility.openClamp();
            sleep(100);
            robot.moveInches(-15, 0.3, 5);

            robot.stop();


        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

    }
}
