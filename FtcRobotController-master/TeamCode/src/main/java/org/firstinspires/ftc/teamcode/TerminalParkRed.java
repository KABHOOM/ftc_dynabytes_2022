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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="redterminalpark")
//@Disabled
public class TerminalParkRed extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Front_left = null;
    private DcMotor Back_left = null;
    private DcMotor Front_right = null;
    private DcMotor Back_right = null;
    //    private DcMotor armMotor = null;
    double lowJunctionPOS=0.51;
    double midJunctionPOS=0.45;
    double highJunctionPOS=0.4;


    static final double INCREMENT   = 0.02;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.3;     // Maximum rotational position
    static final double MIN_POS     =  0.6;     // Minimum rotational position
    static final double MAX_POS_claw     =  0.2;     // Maximum rotational position
    static final double MIN_POS_claw     =  0.0;

    //    boolean kill=false;
    // Define class members
    Servo   servo1arm;
    Servo   servo2arm;
    Servo   servo1claw;
    Servo   servo2claw;
    double  armPosition=MIN_POS;
    double  clawPosition = MIN_POS_claw;


    @Override
    public void runOpMode() throws InterruptedException{

        set_config();
        waitForStart();
        closeClaw();
        leftRightStrafe(800,0.5f);
    }

    public void set_config() {
        //printing for debugging purposes

        System.out.println("Config start");
        Front_left  = hardwareMap.get(DcMotor.class, "front_left_motor");
        Back_left  = hardwareMap.get(DcMotor.class, "back_left_motor");
        Front_right = hardwareMap.get(DcMotor.class, "front_right_motor");
        Back_right = hardwareMap.get(DcMotor.class, "back_right_motor");

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
    //autonomous programs
//if a function calls "continueTill" it must have "throws InterruptedException" as seen below

    public void continueTill(int millis) throws InterruptedException {
        Thread.sleep(millis); //continue for a duration
        //stop all motors
        Front_left.setPower(0);
        Front_right.setPower(0);
        Back_left.setPower(0);
        Back_right.setPower(0);
        //add delay between autonomous actions (to ensure accuracy)
        Thread.sleep(150);
    }

    //power<0 = forward     power>0 = backwards
    public void ForwardBackwards(int millis, float power) throws InterruptedException {
        //set power requested
        Front_left.setPower(power);
        Front_right.setPower(power);
        Back_left.setPower(power);
        Back_right.setPower(power);
        continueTill(millis);
    }

    //power > 0 = left      power < 0 = right
    public void leftRightStrafe(int millis, float power)throws InterruptedException{
        Front_left.setPower(-1 * power);
        Front_right.setPower( power);
        Back_left.setPower( power);
        Back_right.setPower(-1 * power);
        continueTill(millis);
    }

    //power > 0 = left      power < 0 = right           90 degrees turn is millis 840
    public void turnLeftRight(int millis, float power)throws InterruptedException{
        Front_left.setPower(-1 * power);
        Front_right.setPower( power);
        Back_left.setPower(-1 * power);
        Back_right.setPower( power);
        continueTill(millis);
    }


    public void liftArm(){
        if(armPosition>=MAX_POS&&armPosition<=MIN_POS){
            servo1arm.setPosition(armPosition);
            servo2arm.setPosition(armPosition+0.005);
        }
    }

    //arm autonomous


    public void liftArmTeleop(){
        if (gamepad2.left_stick_y>=0.5&&armPosition < MIN_POS) {
            armPosition += 0.01 ;
        }
        else if (gamepad2.left_stick_y<=-0.5&&armPosition>MAX_POS) {
            // Keep stepping down until we hit the min value.
            armPosition -= 0.01 ;
        }
        liftArm();
    }

    public void armLow(){
        armPosition = lowJunctionPOS;
        liftArm();
    }

    public void armMiddle(){
        armPosition = midJunctionPOS;
        liftArm();
    }
    public void armHigh(){
        armPosition = highJunctionPOS;
        liftArm();
    }
    public void armGround()throws InterruptedException{
        while(armPosition<=MIN_POS) {
            armPosition +=0.01;
            liftArm();
            Thread.sleep(100);
        }
    }

    public void moveClawTeleop(){
        if (gamepad2.right_stick_x>=0.5&&clawPosition < MAX_POS_claw) {
            clawPosition += 0.01 ;
        }
        else if (gamepad2.right_stick_x<=-0.5&&clawPosition>MIN_POS_claw) {
            // Keep stepping down until we hit the min value.
            clawPosition -= 0.01 ;
        }
        moveClaw();
    }


    public void moveClaw(){
        servo1claw.setPosition(clawPosition);
        servo2claw.setPosition(clawPosition);
    }

    public void openClaw(){
        clawPosition=MAX_POS_claw;
        moveClaw();
    }

    public void closeClaw(){
        clawPosition=MIN_POS_claw;//becomes 1.05 (does not damage the servo when cone is picked up)
        moveClaw();
    }

    public void autoOpenCloseClawTeleop(){
        if(gamepad2.right_bumper){
            closeClaw();
        }
        if(gamepad2.left_bumper){
            openClaw();
        }
    }
}