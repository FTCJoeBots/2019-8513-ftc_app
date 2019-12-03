package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.List;

import static java.lang.StrictMath.abs;

/**
 * This is NOT an opmode. This is a hardware class used to abstract the hardware config for the
 * 2018 JoeBots FTC Rover Ruckus challenge. This file has been generalized to work as a base for
 * all three JoeBots FTC teams (8513, 11855, and 13702). As the season progresses, this file may be
 * customized for each individual team in their own branch.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * motor0 (left front)
 * motor1 (right front)
 * motor2 (left rear)
 * motor3 (right rear)
 * imu - navigation features
 *
 * Note:  All names are lower case and some have single spaces between words.
 *
 */

public class Robot8513 {
    /* Public OpMode members. */

    HardwareJoeBot2019 robot = new HardwareJoeBot2019();

    // Declare Motors
    public DcMotor liftMotor = null;
    public DcMotor armMotor = null;
    public DcMotor wristMotor = null;
    public Servo capstoneServo = null; //Servo for holding and releasing capstone
   // NormalizedColorSensor colorSensorRight;
   // NormalizedColorSensor colorSensorLeft;

    //declare color sensors
    ColorSensor colorSensorRight;
    DistanceSensor distanceSensorRight;
    ColorSensor colorSensorLeft;
    DistanceSensor distanceSensorLeft;





    float[] hsvValuesRight = {0F, 0F, 0F};
    final float valuesRight[] = hsvValuesRight;
    float[] hsvValuesLeft = {0F, 0F, 0F};
    final float valuesLeft[] = hsvValuesLeft;



    // Declare Servos
    public Servo foundationServo = null; // Servo for foundation
    public Servo clampServo = null; //Servo for grabber

    // Declare Sensors
    //public BNO055IMU imu;                  // The IMU sensor object

    // Variables used for IMU tracking...
    public Orientation angles;
    public Acceleration gravity;
    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Private Members
    private LinearOpMode myOpMode;
    private ElapsedTime runtime = new ElapsedTime();
    private Orientation lastImuAngles = new Orientation();
    private double globalAngle;



    // Declare Static members for calculations
    static final double LIFT_THREADS_PER_INCH = 0.948;
    static final double LIFT_GEAR_REDUCTION = 1;
    static final double LIFT_COUNTS_PER_MOTOR_REV = 145.6;
    static final double LIFT_COUNTS_PER_INCH = (LIFT_THREADS_PER_INCH * LIFT_GEAR_REDUCTION * LIFT_COUNTS_PER_MOTOR_REV);

    static final double WRIST_COUNTS_PER_MOTOR_REV = 4.0;
    static final double WRIST_OUTPUT_COUNTS = 288;


    static final double FOUNDATION_DOWN = 1;
    static final double FOUNDATION_UP = -0.1;

    static final double CLAMP_OPEN = 0.55;
    static final double CLAMP_CLOSE = 1;

    static final int WRIST_MIDDLE = -975; //Wrist parallel to ground
    static final int WRIST_UP = 0; //Wrist up

    static final double CAPSTONE_OPEN = .35;
    static final double CAPSTONE_CLOSE = .85;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        myOpMode = opMode;

        // Define and Initialize Motors
        liftMotor = hwMap.dcMotor.get("liftMotor");
        armMotor = hwMap.dcMotor.get("armMotor");
        wristMotor = hwMap.dcMotor.get("wristMotor");

        capstoneServo = hwMap.servo.get ("capstoneServo");
        foundationServo = hwMap.servo.get("foundationServo");
        clampServo = hwMap.servo.get("clampServo");

        //colorSensorRight = hwMap.NormalizedColorSensor.get("colorSensorRight");
        //colorSensorLeft = hwMap.c
        colorSensorRight = hwMap.colorSensor.get("colorSensorRight") ;
        distanceSensorRight = hwMap.get(DistanceSensor.class, "colorSensorRight") ;


        colorSensorLeft = hwMap.colorSensor.get("colorSensorLeft") ;
        distanceSensorLeft = hwMap.get(DistanceSensor.class, "colorSensorLeft") ;

        foundationServo.setPosition(FOUNDATION_UP);
        clampServo.setPosition(CLAMP_OPEN);
        capstoneServo.setPosition(CAPSTONE_CLOSE);

        // Set Default Motor Directions
        liftMotor.setDirection(DcMotor.Direction.FORWARD); //set to FORWARD (UP) if using AndyMark motors
        armMotor.setDirection(DcMotor.Direction.FORWARD); //set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        liftMotor.setPower(0);
        armMotor.setPower(0);
        wristMotor.setPower(0);
        myOpMode.telemetry.addLine("initialized motor power to zero");
        myOpMode.telemetry.update();


        // Set all drive motors to run without encoders.
        // May want to switch to  RUN_USING_ENCODERS during autonomous
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //Initialize wrist position
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setTargetPosition(WRIST_MIDDLE);
        wristMotor.setPower(0.5);

        if (colorSensorRight instanceof SwitchableLight) {
            ((SwitchableLight) colorSensorRight).enableLight(true);
        }
        if (colorSensorLeft instanceof SwitchableLight) {
                ((SwitchableLight)colorSensorLeft).enableLight(true);
        }

    }

    public void liftMotorInches(double inches, double power) {

        // Declare needed variables
        int newliftMotorTarget;


        // Check to make sure the OpMode is still active; If it isn't don't run the method
        if (myOpMode.opModeIsActive()) {

            // Determine new target positions for each wheel
            newliftMotorTarget = liftMotor.getCurrentPosition() + (int) (inches * LIFT_COUNTS_PER_INCH);

            // Send target Positions to motors
            liftMotor.setTargetPosition(newliftMotorTarget);

            // Set Robot to RUN_TO_POSITION mode
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setPower(power);

            // Reset the runtime
            runtime.reset();

            // Set the motors back to standard mode
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //Move arm forward and backward
    public void ExtendArmInches(double inches, double power) {

        // Declare needed variables
        int newarmMotorTarget;

        // Check to make sure the OpMode is still active; If it isn't don't run the method
        if (myOpMode.opModeIsActive()) {

            // Determine new target positions for each wheel
            newarmMotorTarget = liftMotor.getCurrentPosition() + (int) (inches * LIFT_COUNTS_PER_INCH);

            // Send target Positions to motors
            armMotor.setTargetPosition(newarmMotorTarget);

            // Set Robot to RUN_TO_POSITION mode
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armMotor.setPower(power);

            // Reset the runtime
            runtime.reset();

            // Set the motors back to standard mode
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    // grabs the foundation
    public void grabFoundation() {

        foundationServo.setPosition(FOUNDATION_DOWN);
    }

    // releases the foundation
    public void releaseFoundation() {

        foundationServo.setPosition(FOUNDATION_UP);


    }

    // opens servo for clamp
    public void openClamp() {

        clampServo.setPosition(CLAMP_OPEN);
    }
    // closes servo for clamp
    public void closeClamp () {

        clampServo.setPosition(CLAMP_CLOSE);


    }

    public void wristFlat(double power){
        if (myOpMode.opModeIsActive()) {

            wristMotor.setTargetPosition(WRIST_MIDDLE);
            wristMotor.setPower(power);
        }
    }

    public void wristPosition(double power) {

        // This method should take in the operator control (via the power variable) which will
        // control both how fast, and which direction the wrist moves.

        // There is no need to take in "wristPos" here because we can read that from the
        // wristMotor at any time.


        // Since our "FLAT" position is negative, we want to DECREASE encoder values as the wrist
        // moves down...

        // Also, we're not going to map the stick directly to the wristPower. Essentially, if the stick
        // is less than halfway moved, we'll use a low power movement, and a high power movement for
        // more than half.

        int newTargetPos;
        double newPower;

        if (power < -0.5) {
            // move down fast
            newTargetPos = wristMotor.getCurrentPosition() - 100;
            newPower = 0.6;
        } else if (power < 0 ) {
            // move down slowly
            newTargetPos = wristMotor.getCurrentPosition() - 30;
            newPower = 0.3;
        } else if (power > 0.5) {
            // move up quickly
            newTargetPos = wristMotor.getCurrentPosition() + 100;
            newPower = 0.6;
        } else if (power > 0) {
            // move up slowly
            newTargetPos = wristMotor.getCurrentPosition() + 30;
            newPower = 0.3;
        } else {
            // power is 0
            newTargetPos = wristMotor.getCurrentPosition();
            newPower = 0.3;
        }

        // Check for min and max

        if (newTargetPos > WRIST_UP) {
            // We can't go higher than WRIST_UP
            newTargetPos = WRIST_UP;
        }

        if (newTargetPos < WRIST_MIDDLE) {
            // we don't want to go below flat
            newTargetPos = WRIST_MIDDLE;
        }

        // Apply Power and Target Position

        wristMotor.setTargetPosition(newTargetPos);
        wristMotor.setPower(newPower);


    }

    public void capstoneOpen() {

        capstoneServo.setPosition(CAPSTONE_OPEN);
    }

    public void capstoneClose() {

        capstoneServo.setPosition(CAPSTONE_CLOSE);
    }


    /* Initialize standard Hardware interfaces */
    public void Autoinit(HardwareMap ahwMap, LinearOpMode opMode) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        myOpMode = opMode;

        // Define and Initialize Motors
        liftMotor = hwMap.dcMotor.get("liftMotor");
        armMotor = hwMap.dcMotor.get("armMotor");
        wristMotor = hwMap.dcMotor.get("wristMotor");

        foundationServo = hwMap.servo.get("foundationServo");
        clampServo = hwMap.servo.get("clampServo");


        foundationServo.setPosition(FOUNDATION_UP);
        clampServo.setPosition(CLAMP_OPEN);


        // Set Default Motor Directions
        liftMotor.setDirection(DcMotor.Direction.FORWARD); //set to FORWARD (UP) if using AndyMark motors
        armMotor.setDirection(DcMotor.Direction.FORWARD); //set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        liftMotor.setPower(0);
        armMotor.setPower(0);
        wristMotor.setPower(0);
        myOpMode.telemetry.addLine("initialized motor power to zero");
        myOpMode.telemetry.update();


        // Set all drive motors to run without encoders.
        // May want to switch to  RUN_USING_ENCODERS during autonomous
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //Initialize wrist position
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //wristMotor.setTargetPosition(WRIST_MIDDLE);
        //wristMotor.setPower(0.5);




    }
public int isYellow
        (){

    //NormalizedRGBA colorsRight = sensorColorRight.getNormalizedColors();
    //NormalizedRGBA colorsLeft = sensorColorLeft.getNormalizedColors();

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;


    Color.RGBToHSV((int)(colorSensorRight.red() * SCALE_FACTOR),
            (int) (colorSensorRight.green() * SCALE_FACTOR),
            (int) (colorSensorRight.blue() * SCALE_FACTOR),
            hsvValuesRight);
    Color.RGBToHSV((int)(colorSensorLeft.red() * SCALE_FACTOR),
            (int) (colorSensorLeft.green() * SCALE_FACTOR),
            (int) (colorSensorLeft.blue() * SCALE_FACTOR),
            hsvValuesLeft);



    return 0;
}
    public double [] distance ()
    {
        double distanceArray [] = {-1,-1};
        distanceArray[0] = distanceSensorRight.getDistance(DistanceUnit.CM);
        distanceArray[1] = distanceSensorLeft.getDistance(DistanceUnit.CM);

        return distanceArray;

    }
}








