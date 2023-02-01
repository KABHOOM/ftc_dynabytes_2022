/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.openCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

//@TeleOp
@Autonomous(name="Sleeve Detection Left")
public class sleeveDetection extends DBTRobot
{

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    // Sleeve TAG IDs
    int LEFT = 3;
    int MIDDLE = 4;
    int RIGHT = 5;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        set_config();



        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        scoreHighJunction();
        /* Actually do something useful */
        if(tagOfInterest == null)
        {


            //get come from stack and put on low junction

            //be parallel to the small pole
            turnRobot(500,0.5f);
            sleep(500);
            moveRobot(650,0.5f,false);
            sleep(500);


            coneFromStack();

            telemetry.addLine("Not found");
            telemetry.update();
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */


            // e.g.
            if(tagOfInterest.id == LEFT)
            {
                // do something
                telemetry.addLine("LEFT");
                telemetry.update();
                moveRobot(1400,0.5F,true);
                moveRobot(100,-0.5F,false);
            }
            else if(tagOfInterest.id==RIGHT)
            {
                // do something else
                telemetry.addLine("RIGHT");
                telemetry.update();
                moveRobot(650,-0.5F,true);
            }
            else if(tagOfInterest.id==MIDDLE)
            {
                // do something else
                moveRobot(500,0.5F,true);


            }
        }


       // while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public void coneFromStack(){
        armPosition=0.56;
        liftArm();
        sleep(500);
        openClaw();
        sleep(500);
        moveRobot(200,0.5f,false);
        sleep(500);
        closeClaw();
        sleep(500);
        armPosition+=0.01;
        liftArm();
        sleep(500);
        moveRobot(25,-0.5f,false);
        sleep(500);
        liftArmMiddle();
        sleep(500);
        moveRobot(650,-0.5f,false);
        sleep(500);
        turnRobot(400,0.5f);
        sleep(500);
        liftArmLow();
        sleep(500);
        moveRobot(200,0.5f,false);
        sleep(500);
        openClaw();
        sleep(500);
        closeClaw();
        moveRobot(200,-0.5f,false);
        sleep(500);
        moveRobot(500,0.5f,true);
    }
    public void scoreHighJunction(){

        closeClaw();
        sleep(200);
        liftArm();
        sleep(200);


        // Move forward to detach from field wall
        moveRobot(50, 1.0f, false);
        sleep(500);
        // Move right
        moveRobot(650, -0.7f, true);
        sleep(500);
        //readjust robots direction
        turnRobot(17, -1.0f);
        sleep(500);
        // Move forward
        moveRobot(750, 0.7f, false);
        sleep(1000);
        // Move left
        moveRobot(325, 0.7f, true);
        sleep(1000);

        turnRobot(25, 1.0f);
        sleep(400);

        moveRobot(110, 0.7f, true);
        sleep(1000);

        // Move arm to high junction
        liftArmHigh();
        sleep(2000);
        // Move forward a bit

        moveRobot(550, 0.2f, false);
        sleep(500);
        //align robot with junction
//        turnRobot(8, -1.0f);
//        sleep(500);
        // Open claw

        openClaw();
        sleep(500);
        // Close claw
        closeClaw();
        sleep(100);
        // Move back
        moveRobot(650, -0.2f, false);
        sleep(500);
        //arm to ground
        liftArmGround();
        sleep(500);
        //recenter robot
        turnRobot(120, 0.5f);
        sleep(400);


    }
}

class DBTRobot extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Front_left = null;
    private DcMotor Back_left = null;
    private DcMotor Front_right = null;
    private DcMotor Back_right = null;
    //    private DcMotor armMotor = null;
    double lowJunctionPOS=0.51;
    double midJunctionPOS=0.45;
    double highJunctionPOS=0.39;


    static final double INCREMENT   = 0.02;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.3;     // Maximum rotational position for arm
    static final double MIN_POS     =  0.61;     // Minimum rotational position for arm
    static final double MAX_POS_claw     =  0.2;     // Maximum rotational position for claw
    static final double MIN_POS_claw     =  0.0;  // Minimum rotational position for claw

    //    boolean kill=false;
    // Define class members
    Servo servo1arm; // servo motor for right arm
    Servo   servo2arm; // servo motor for left arm
    Servo   servo1claw; // servo motor for right claw
    Servo   servo2claw; // servo motor for left claw
    double  armPosition=0.59;
    double  clawPosition = MIN_POS_claw;


    @Override
    public void runOpMode() throws InterruptedException{

    }

    public void set_config() {
        /*
        This function set configuration matching to driver controller configuration.
        Any addition or modification of harward configuration can be directly done here

         */

        System.out.println("Config start");
        Front_left  = hardwareMap.get(DcMotor.class, "front_left_motor");
        Back_left  = hardwareMap.get(DcMotor.class, "back_left_motor");
        Front_right = hardwareMap.get(DcMotor.class, "front_right_motor");
        Back_right = hardwareMap.get(DcMotor.class, "back_right_motor");

        // setting up motor direction
        Front_left.setDirection(DcMotor.Direction.REVERSE);
        Back_left.setDirection(DcMotor.Direction.REVERSE);
        Front_right.setDirection(DcMotor.Direction.FORWARD);
        Back_right.setDirection(DcMotor.Direction.FORWARD);

        servo1claw = hardwareMap.get(Servo.class, "right_claw");
        servo2claw = hardwareMap.get(Servo.class, "left_claw");
        servo2claw.setDirection(Servo.Direction.REVERSE);

        servo1arm = hardwareMap.get(Servo.class, "right_arm");
        servo2arm = hardwareMap.get(Servo.class, "left_arm");
        servo1arm.setDirection(Servo.Direction.REVERSE);
        System.out.println("Config end");
    }

    // Robot functions to use during teleop
    public void moveRobot_in_teleOp_mode(){
        /*
        This function allow robot to run in teleop mode where gamepad 1 joy sticks (both left and right)
        can control robot movement
         */
        double max;
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels
        Front_left.setPower(0.5*leftFrontPower);
        Front_right.setPower(0.5*rightFrontPower);
        Back_left.setPower(0.5*leftBackPower);
        Back_right.setPower(0.5*rightBackPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();

        // Movement code with gamepad ends
    }
    public void moveClaw_in_teleOp_mode(){
        // This function helps move claw with gamepad 2 right stick
        if (gamepad2.right_stick_x>=0.5&&clawPosition < MAX_POS_claw) {
            clawPosition += 0.01 ;
        }
        else if (gamepad2.right_stick_x<=-0.5&&clawPosition>MIN_POS_claw) {
            // Keep stepping down until we hit the min value.
            clawPosition -= 0.01 ;
        }
        moveClaw();
    }
    public void liftArmTeleop(){
        // This function helps move lift arm with gamepad 2 left stick
        if (gamepad2.left_stick_y>=0.5&&armPosition < MIN_POS) {
            armPosition += 0.01 ;
        }
        else if (gamepad2.left_stick_y<=-0.5&&armPosition>MAX_POS) {
            // Keep stepping down until we hit the min value.
            armPosition -= 0.01 ;
        }
        liftArm(); // lift the arm to arm position
    }


    //Robot basic functions to use either during teleop or autonomus to control robot
    public void moveRobot(int milliseconds, float power, boolean strafe){
        /* This function is for autonomus mode to move Robot forward or backward
        INPUT:
            - milliseconds : its time in milliseconds.
                             Robot will move forward or backward for specified  milliseconds
                             values should be in integer only (no decimals)
                   - power : its power for robot movement.
                             Robot will move forward or backward with specified  power
                             power should be between -1 and 1.
                             Negative power move robot backward or strafe LEFT
                             Positive power move robot forward or strafe RIGHT


        */
        setRobotPower(power,strafe); // moving robot with specified power
        sleep(milliseconds); // keeping robot move for specified duration
        setRobotPower(0,false); // stopping robot from further moving


    }
    public void turnRobot(int milliseconds, float power){
        /* This function is for autonomus mode to turn Robot LEFT or RIGHT
        //power > 0 = left      power < 0 = right           90 degrees turn is millis 840
        INPUT:
            - milliseconds : its time in milliseconds.
                             Robot will turn LEFT or RIGHT for specified  milliseconds
                             values should be in integer only (no decimals)
                   - power : its power for robot turn.
                             Robot will turn LEFT or RIGHT with specified  power
                             power should be between -1 and 1.
                             Negative power turn robot in RIGHT direction
                             Positive power turn robot in LEFT direction


        */
        //setting up power for robot turn
        Front_left.setPower(-1 * power);
        Front_right.setPower( power);
        Back_left.setPower(-1 * power);
        Back_right.setPower( power);

        sleep(milliseconds); // keeping robot move for specified duration
        setRobotPower(0,false); // stopping robot from further moving

    }
    public void liftArmLow(){
        armPosition = lowJunctionPOS;
        liftArm();
    }
    public void liftArmMiddle(){
        armPosition = midJunctionPOS;
        liftArm();
    }
    public void liftArmHigh(){
        armPosition = highJunctionPOS;
        liftArm();
    }
    public void liftArmGround(){
        // This function set lift servo position for ground.
        // Loop introduced to reduce speed of arm going down to ground and avoid accident or slippage.
        while(armPosition<=MIN_POS) {
            armPosition +=0.05;
            liftArm();
            //sleep(10);
        }
    }
    public void openClaw(){
        // This function open claw fully and set max claw position
        clawPosition=MAX_POS_claw;
        moveClaw();
    }
    public void closeClaw(){
        // This function close claw fully and set claw position to minimum
        clawPosition=MIN_POS_claw;//becomes 1.05 (does not damage the servo when cone is picked up)
        moveClaw();
    }

    // supportive functions
    public void liftArm(){
        // This function set position of lift arm servo motors if position are in range of MAX and MIN position
        if(armPosition>=MAX_POS&&armPosition<=MIN_POS){
            servo1arm.setPosition(armPosition);
            servo2arm.setPosition(armPosition+0.01);
        }
    }
    public void moveClaw(){
        // setting up claw position
        servo1claw.setPosition(clawPosition);
        servo2claw.setPosition(clawPosition);
    }
    public void setRobotPower(float power,boolean strafe){
        /*
        This function sets power to all motor/wheels with specified power
        INPUT:
            power : its power for robot movement.
                    Robot will move forward or backward with specified  power
                    power should be between -1 and 1.
                    Negative power move robot backward or strafe LEFT
                    Positive power move robot forward or strafe RIGHT

            strafe: it's flag for strafe.
                    FALSE value will not allow robot to strafe
                    TRUE value will allow robot to strafe LEFT and Right with specified power direction
        */
        //power = power; // Reversing power due to configuration as  power<0 = forward  & power>0 = backwards.

        if (strafe){
            // When Strafe is TRUE set power accordingly to strafe robot in LEFT or RIGHT
            Front_left.setPower(-1* power);
            Back_right.setPower(-1* power);
        }
        else {
            // When Strafe is FALSE set power accordingly to strafe robot in FORWARD or BACKWARD
            Front_left.setPower(power);
            Back_right.setPower(power);
        }

        Front_right.setPower(power);
        Back_left.setPower(power);



    }


}