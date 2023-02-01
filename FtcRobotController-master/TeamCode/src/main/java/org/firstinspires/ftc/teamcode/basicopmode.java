/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.

 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="POV mode", group="Linear Opmode")
//@Disabled
public class basicopmode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Front_left = null;
    private DcMotor Back_left = null;
    private DcMotor Front_right = null;
    private DcMotor Back_right = null;
    //    private DcMotor armMotor = null;
    double lowJunctionPOS=0.5;
    double midJunctionPOS=0.45;
    double highJunctionPOS=0.36;


    static final double INCREMENT   = 0.02;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.33;     // Maximum rotational position for arm
    static final double MIN_POS     =  0.62;     // Minimum rotational position for arm
    static final double MAX_POS_claw     =  0.2;     // Maximum rotational position for claw
    static final double MIN_POS_claw     =  0.0;  // Minimum rotational position for claw

    //    boolean kill=false;
    // Define class members
    Servo   servo1arm; // servo motor for right arm
    Servo   servo2arm; // servo motor for left arm
    Servo   servo1claw; // servo motor for right claw
    Servo   servo2claw; // servo motor for left claw
    double  armPosition=MIN_POS;
    double  clawPosition = MIN_POS_claw;


    @Override
    public void runOpMode() throws InterruptedException{
        // This is the main function

        /* setting configuration with corresponding driver configuration through set_config functions.
        for any addition or modification of configuration please update set_config function with required
        changes
        */
        set_config();

        // Dispalying all current value of servo motors (arms & claw) on driver controller
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.addData("Servo 1 arm Position (actual)\n", "%5.2f", servo1arm.getPosition());
        telemetry.addData("Servo 2 arm Position (actual)\n", "%5.2f", servo2arm.getPosition());
        telemetry.addData("Servo 1 claw Position (actual)\n", "%5.2f", servo1claw.getPosition());
        telemetry.addData("Servo 2 claw Position (actual)\n", "%5.2f", servo2claw.getPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Wait for the start button
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
                This is the main loop function which goes on loop until opMode ia active.
                This is where all game control logic can be written for both autonomus or tele-op operations.
            */

            // move robot with gamepad 1 left and right stick position
            moveRobot_in_teleOp_mode();

            // move left arm with gamepad 2 left stick
            liftArmTeleop();

            // move both left and right claw with gamepad 2 right stick
            moveClaw_in_teleOp_mode();


            /*
            preset arm positions with gamepad 2 button to go to the height of a low, medium, or high junction
            Got to
                - Low junction height  "A" button press on gamepad 2
                - Medium junction height with "B" button press on gamepad 2
                - High junction height with "Y" button press on gamepad 2
                - Ground height with "X" button press on gamepad 2
            */
            //high junction
            if(gamepad2.y==true){
                liftArmHigh(); // function takes lift arm to High junction height
            }
            //medium junction
            if(gamepad2.b==true){
                liftArmMiddle(); // function takes lift arm to Medium junction height
            }
            //low junction
            if(gamepad2.a==true){
                liftArmLow(); // function takes lift arm to Low junction height
            }
            //ground
            if(gamepad2.x==true){
                closeClaw(); // before going ground claw needs to be closed in order to avoid accident
                liftArmGround(); // function takes lift arm to Ground junction height
            }


            //close claw with gamepad 2 right bumper button
            if(gamepad2.right_bumper){
                closeClaw();
            }
            // open claw with gamepade 2 left bumper button press
            if(gamepad2.left_bumper){
                openClaw();
            }

            // Display the current value

            telemetry.addData("Servo 1 Position (code)", "%5.2f", armPosition);
            telemetry.addData("Servo 1 Position (actual)\n", "%5.2f", servo1arm.getPosition());
            telemetry.addData("Servo 2 Position (code)", "%5.2f", armPosition);
            telemetry.addData("Servo 2 Position (actual)\n", "%5.2f", servo2arm.getPosition());
            telemetry.update();

            sleep(CYCLE_MS);
            idle();

            // Signal done;
            telemetry.addData(">", "Done");
            telemetry.update();

        }
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
    public void turnRobot(int milliseconds, float power)throws InterruptedException{
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
        power = -1*power; // Reversing power due to configuration as  power<0 = forward  & power>0 = backwards.

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