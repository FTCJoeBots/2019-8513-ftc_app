

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BlueFoundationAuto2019-3", group="JoeBot")
//@Disabled
public class parkingAndFoundation3Blue8513 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareJoeBot2019 robot = new HardwareJoeBot2019();
    Robot8513 utility = new Robot8513();
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);
        utility.Autoinit(hardwareMap, this);

        waitForStart();

        robot.moveInches(-49.5,.25,7);
        sleep(300);
        robot.strafeSeconds(400, .5);
        sleep(300);
        utility.grabFoundation();
        sleep(500);
        robot.strafeSeconds(400, .3);
        sleep(300);
        robot.moveInches(86, .25, 15);
        sleep(700);

        robot.strafeSeconds(800, -.3);

        robot.moveInches(12, 0.3, 7);

        sleep(500);

        utility.releaseFoundation();
        sleep(100);

        robot.strafeSeconds(100, -.3);

        sleep(500);



        robot.stop();

    }


}
