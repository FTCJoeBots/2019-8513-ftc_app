package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Image_Recognition {

    HardwareMap hwMap = null;
    private LinearOpMode myOpMode;
    int cameraMonitorViewId;
    VuforiaTrackables targetsSkyStone;
    private static final float mmPerInch = 25.4f;

    private static final float stoneZ = 2.00f * mmPerInch;

    final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    final boolean PHONE_IS_PORTRAIT = true;


    final String VUFORIA_KEY =
            "AVzCl0v/////AAAAGcfsmNB0+Ecxi9nnFUli4RtGGZORFTsrkrZTsSaEZcnHNkxhb5NbskfqT531gL1cmgLFZ5xxeICDdBlPxxEbD4JcUvUuIdXxpVesR7/EAFZ+DTSJT3YQb0sKm2SlOlfiMf7ZdCEUaXuymCZPB4JeoYdogDUOdsOrd0BTDV2Z+CtO3eSsHWfcY6bDLh8VJKSbeFdk533EzcA26uhfhwBxYlzbOsjPSVCB66P6GbIP9/UjI3lbTNi+tpCpnOZa2gwPjoTSeEjo9ZKtkPe3a/DpLq3OMnVwVnUmsDvoW++UbtOmg9WNFC/YkN7DCtMt91uPaJPL5vOERkA+uXliC1i44IT4EyfoN1ccLaJiXMFH63DE";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia = null;
    VuforiaTrackable stoneTarget;
    VuforiaLocalizer.Parameters parameters;
    boolean targetVisible = false;
    float phoneXRotate = 0;
    float phoneYRotate = 0;
    float phoneZRotate = 0;

    public void init(HardwareMap ahwMap, LinearOpMode opMode) {

        opMode.telemetry.addLine("Starting init");
        opMode.telemetry.update();
        myOpMode = opMode;
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.useExtendedTracking = true;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        myOpMode.telemetry.addLine("defining trackables");
        myOpMode.telemetry.update();
        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
        myOpMode.telemetry.addLine("finishing init");
        myOpMode.telemetry.update();
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        myOpMode.telemetry.addLine("stone target defined");
        myOpMode.telemetry.update();
        myOpMode.telemetry.addLine("Ending init");
        myOpMode.telemetry.update();
        myOpMode.telemetry.addLine(" Starting Skysone Postion");
        myOpMode.telemetry.update();
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


    }

    /*public double SkystonePostion() {

        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
        myOpMode.telemetry.addLine("finishing init");
        myOpMode.telemetry.update();
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        myOpMode.telemetry.addLine("stone target defined");
        myOpMode.telemetry.update();
        myOpMode.telemetry.addLine("Ending init");
        myOpMode.telemetry.update();
        myOpMode.telemetry.addLine(" Starting Skysone Postion");
        myOpMode.telemetry.update();
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        // For convenience, gather together all the trackable objects in one easily-iterable collection
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);


// We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        myOpMode.telemetry.addLine("Seting up open Gl matrix");
        myOpMode.telemetry.update();
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }


        targetsSkyStone.activate();
        while (!myOpMode.isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    myOpMode.telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                myOpMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0), translation.get(1), translation.get(2));
                if (translation.get(1) > -180) {
                    myOpMode.telemetry.addLine("First Postion");
                }
                if (translation.get(1) < -180) {
                    myOpMode.telemetry.addLine("Seccond Position");
                }


            } else {
                myOpMode.telemetry.addLine("Third Position");
            }
            myOpMode.telemetry.update();
        }

        return 0;
    }
**/

    public double[] skystone_cooridinates() {
        double[] coords = {-999, -999};
        double [] nf_coords = {777,777};
/*        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
        myOpMode.telemetry.addLine("finishing init");
        myOpMode.telemetry.update();
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        myOpMode.telemetry.addLine("stone target defined");
        myOpMode.telemetry.update();
        myOpMode.telemetry.addLine("Ending init");
        myOpMode.telemetry.update();
        myOpMode.telemetry.addLine(" Starting Skysone Postion");
        myOpMode.telemetry.update();
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        // For convenience, gather together all the trackable objects in one easily-iterable collection
*/
        myOpMode.telemetry.addLine("seting up array");
        myOpMode.telemetry.update();

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//problem is here
        allTrackables.addAll(targetsSkyStone);

        myOpMode.telemetry.addLine("Array set up");
        myOpMode.telemetry.update();
// We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        myOpMode.telemetry.addLine("Seting up open Gl matrix");
        myOpMode.telemetry.update();
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }


        targetsSkyStone.activate();
        while (!myOpMode.isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    myOpMode.telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                myOpMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0), translation.get(1), translation.get(2));
                myOpMode.telemetry.update();
                coords[0] = translation.get(0);
                coords[1] = translation.get(1);
                return coords;

            }
            ///return a junk value here
            return nf_coords;
        }

        return coords;
    }


}



