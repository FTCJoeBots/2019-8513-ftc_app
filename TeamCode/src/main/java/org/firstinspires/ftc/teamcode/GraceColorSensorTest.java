
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 *
 * This is a test Autonomous code to check the workings of the "moveInches" and "rotate" commands
 * in the 2018 HardwareJoeBots class.
 *
 */

@TeleOp(name="Grace Color Sensor Test", group="8513")
//@Disabled
public class GraceColorSensorTest extends LinearOpMode {

  /* Declare OpMode members. */
  HardwareJoeBot2019 robot = new HardwareJoeBot2019();
  Robot8513 utility = new Robot8513();



 // sensorColor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

 // sensorColor = HardwareMap.ColorSensor.get("color");
  //DistanceSensor sensorDistance;
  float hsvValuesRight[] = {0F, 0F, 0F};
  float hsvValuesLeft[] = {0F, 0F, 0F};



  @Override
  public void runOpMode() {

    /*
     * Initialize the drive system variables.
     * The init() method of the hardware class does all the work here
     */
    robot.init(hardwareMap, this);
    utility.init(hardwareMap, this);

    int SCALE_FACTOR = 255;

    // Send telemetry message to signify robot waiting;
    telemetry.addData("Status", "Resetting Encoders");    //
    telemetry.update();

    robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    Color.RGBToHSV((int) (utility.colorSensorRight.red() * SCALE_FACTOR),
              (int) (utility.colorSensorRight.green() * SCALE_FACTOR),
              (int) (utility.colorSensorRight.blue() * SCALE_FACTOR),
              hsvValuesRight);

    Color.RGBToHSV((int) (utility.colorSensorLeft.red() * SCALE_FACTOR),
              (int) (utility.colorSensorLeft.green() * SCALE_FACTOR),
              (int) (utility.colorSensorLeft.blue() * SCALE_FACTOR),
              hsvValuesLeft);

      while (opModeIsActive()) {
        telemetry.addLine("VALUE     RIGHT   LEFT");
        telemetry.addData("Red:   ", "%3d   %3d", utility.colorSensorRight.red(), utility.colorSensorLeft.red());
        telemetry.addData("Green: ", "%3d   %3d", utility.colorSensorRight.green(), utility.colorSensorLeft.green());
        telemetry.addData("Blue:  ", "%3d   %3d", utility.colorSensorRight.blue(), utility.colorSensorLeft.blue());
        telemetry.addData("Alpha: ", "%3d   %3d", utility.colorSensorRight.alpha(), utility.colorSensorLeft.alpha());
        telemetry.addData("Hue:   ", "%3f   %3f", hsvValuesRight[0], hsvValuesLeft[0]);
        telemetry.addData("Sat:   ", "%3f   %3f", hsvValuesRight[1], hsvValuesLeft[1]);
        telemetry.addData("Distance: ", "%.02f   %.02f",
                utility.distanceSensorRight.getDistance(DistanceUnit.CM),
                utility.distanceSensorLeft.getDistance(DistanceUnit.CM));

        telemetry.update();
      }



  }
}