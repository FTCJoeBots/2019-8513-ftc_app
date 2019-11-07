
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 *import com.qualcomm.robotcore.hardware.DcMotor;
 *
 *
 */

/**
 *Notes For this TeleOp Code. This code is for Comp and all proggramers should review over this
 *code and understand this code for the possibility that a question may be asked related to TeleOp and
 *you should be able to explain in good detail everything in this code.
 *11/16/17-> Changed all gamepad's in code to correct gamepad (i.e some gamepad1's to gamepad2)
 ***11/18/17-> Competition Notes below
 *Notes-> Autonomous is incorrect, Not much was wrong from a software sandpoint but hardware issues were fixed
 *Autonomous issues included: Incorrect spinning causing us to move out of destination,
 *To much time on the down motion of the clamp and arm.
 *These issues are still not resolved
 * Recomendation for autonomous issues(Not Offical):Fine tune the timer on the clamp
 * Fine tune the movements and LOWER the TIME OF MOVEMENT in autonomous.
 * List of issues at Comp(1)-> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1r_liipKBU7GHfONdxq9E6d4f7zikcCuXwDL2bsQfwm0/edit?usp=sharing
 *G-Sheet of time VS Heading for autonomous -> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1pqv0iN94fFd5KvX1YIWP7z39HgpURXsscn0zPujs1q4/edit?usp=sharing
 */
@TeleOp(name="Not this one", group="TeleOp")

public class NewTeleop extends LinearOpMode {

    double forward;
    double clockwise;
    double right;
    double k;
    double power0;
    double power1;
    double power2;
    double power3;
    double max;
    double liftPower;
    double armPower;
    double wristPower;
   // double lift; //lift arm up
   // double extendPower; //extend arm forward
    double clampOpen;
    double clampClose;
    boolean CurrFoundGrabStateB = false;
    boolean PrevFoundGrabStateB = false;
    boolean CurrFoundReleaseStateA = false;
    boolean PrevFoundReleaseStateA = false;
    boolean CurrClampOpenY = false;
    boolean PrevClampOpenY = false;
    boolean CurrClampCloseX = false;
    boolean PrevclampCloseX = false;
    boolean wristStraight = false;

    // double lift; //lift arm up
    // double extendPower; //extend arm forward
    double clamp;
    double foundation;



    HardwareJoeBot2019 robot = new HardwareJoeBot2019();
    Robot8513 utility = new Robot8513();

    @Override
    public void runOpMode() throws InterruptedException {




        robot.init(hardwareMap, this);
        utility.init(hardwareMap, this);


        waitForStart();



        //start of loop
        while (opModeIsActive()) {


            //Drive Via "Analog Sticks" (Not Toggle)
            //Set initial motion parameters to Gamepad1 Inputs
            forward = -gamepad1.left_stick_y;
            //right = gamepad1.left_stick_x;
            right = -gamepad1.left_trigger + gamepad1.right_trigger;
            clockwise = gamepad1.right_stick_x;
            armPower = -gamepad2.right_stick_x; //arm forward and backward
            liftPower = gamepad2.left_stick_y; //lift up and down

            wristPower = -gamepad2.left_stick_x; //move wrist up and down


            CurrClampOpenY = gamepad2.y;
            CurrClampCloseX = gamepad2.x;
            CurrFoundReleaseStateA = gamepad2.a;
            CurrFoundGrabStateB = gamepad2.b;



            // Add a tuning constant "K" to tune rotate axis sensitivity
            k = .6;
            clockwise = clockwise * k; //Make sure the "= Clockwise" is "= -clockwise"


            // Calculate motor power
            power0 = forward + clockwise + right;
            power1 = forward - clockwise - right;
            power2 = forward + clockwise - right;
            power3 = forward - clockwise + right;


            // Normalize Wheel speeds so that no speed exceeds 1.0

            max = Math.abs(power0);
            if (Math.abs(power1) > max) {
                max = Math.abs(power1);
            }
            if (Math.abs(power2) > max) {
                max = Math.abs(power2);
            }
            if (Math.abs(power3) > max) {
                max = Math.abs(power3);
            }

            if (max > 0.75) {
                power0 /= max;
                power1 /= max;
                power2 /= max;
                power3 /= max;
            }

            robot.motor0.setPower(power0);
            robot.motor1.setPower(power1);
            robot.motor2.setPower(power2);
            robot.motor3.setPower(power3);
            utility.liftMotor.setPower(liftPower); //lift up and down the arm
            utility.armMotor.setPower(armPower); //Extend and contract the arm
            utility.wristMotor.setPower(wristPower);

            //Turn wrist up and down
            int CurPos = utility.wristMotor.getCurrentPosition();
            utility.wristPosition(CurPos,wristPower);

            // Update Telemetry
            telemetry.addData(">", "Press Stop to end test.");

            if (gamepad2.right_bumper) {
                utility.wristFlat(.5);
            }

            if (gamepad1.a) {
                telemetry.addLine("Button A is pressed");
            } else if (gamepad1.b) {
                telemetry.addLine("Button B is pressed");
            } else {
                telemetry.addLine("Neither button is pressed");
            }


            if (clampOpen > 0) {
                utility.openClamp();

            } else if (clampClose > 0) {
                utility.closeClamp();
            }

            CurrFoundGrabStateB = gamepad2.b;
            if ((CurrFoundGrabStateB == true) && (CurrFoundGrabStateB != PrevFoundGrabStateB)) {

                // When the "B" button is pressed, close foundation

                utility.grabFoundation();

            }
            PrevFoundGrabStateB = CurrFoundGrabStateB;

            CurrFoundReleaseStateA = gamepad2.a;
            if ((CurrFoundReleaseStateA == true) && (CurrFoundReleaseStateA != PrevFoundReleaseStateA)) {

                // When the "A" button is pressed, open foundation

                utility.releaseFoundation();
            }
            PrevFoundReleaseStateA = CurrFoundReleaseStateA;

            CurrClampCloseX = gamepad2.x;
            if ((CurrClampCloseX == true) && (CurrClampCloseX != PrevclampCloseX)) {

                // When the "X" button is pressed, close clamp

                utility.closeClamp();

            }
            PrevclampCloseX = CurrClampCloseX;

            CurrClampOpenY = gamepad2.y;
            if ((CurrClampOpenY == true) && (CurrClampOpenY != PrevClampOpenY)) {

                // When the "Y" button is pressed, open clamp

                utility.openClamp();
            }
            PrevClampOpenY = CurrClampOpenY;




            telemetry.update();
            idle();

            telemetry.addLine();




        }//end while
    }
}

