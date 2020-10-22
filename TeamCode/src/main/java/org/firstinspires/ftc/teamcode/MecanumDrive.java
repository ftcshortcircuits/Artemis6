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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MecanumDrive", group="Linear Opmode")
@Disabled
public class MecanumDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Declaring our motors.
    private DcMotor LeftBack;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private Servo FrontHook;
    private Servo BackHook;

    //Declared variables for speed.
    private double LeftBackSpeed;
    private double RightBackSpeed;
    private double LeftFrontSpeed;
    private double RightFrontSpeed;

    //Declared variables for translation, rotation, and scale factor.
    private double translatex;
    private double translatey;
    private double rotation;
    public int Scale;

    /*
     * Adjustable parameters for the robot.  Add new mechanism-related constants here.
     */
    private double FRONT_HOOK_PARK          = 0.1 ;         // Park location (full up) for servo hook
    private double FRONT_HOOK_DOWN          = 0.65 ;        // Foundation drag position for servo hook
    private double FRONT_HOOK_STONE         = 0.57 ;        // Stone drag position for servo hook
    private double BACK_HOOK_PARK           = 0.95 ;        // Park location (full up) for servo hook
    private double BACK_HOOK_DOWN           = 0.35 ;        // Foundation drag position for servo hook
    private double BACK_HOOK_STONE          = 0.45 ;        // Stone drag position for servo hook

    private double DEADZONE                 = 0.05 ;        // Center deadzone for gamepad sticks

    @Override
    public void runOpMode() {
        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initializing motors.
        LeftBack  = hardwareMap.get(DcMotor.class, "lBack");
        RightBack  = hardwareMap.get(DcMotor.class, "rBack");
        LeftFront  = hardwareMap.get(DcMotor.class, "lFront");
        RightFront  = hardwareMap.get(DcMotor.class, "rFront");
        FrontHook =  hardwareMap.get(Servo.class,  "fHook");
        BackHook = hardwareMap.get(Servo.class, "bHook");

        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftBack.setPower(0);
        LeftFront.setPower(0);
        RightBack.setPower(0);
        RightFront.setPower(0);
        FrontHook.setPosition(FRONT_HOOK_PARK);
        BackHook.setPosition(BACK_HOOK_PARK);

        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            * This chunk helps us take in inputs. It is written in this way to make it
            * condensed and concise.
             */

            translatex = gamepad1.left_stick_x;
            translatey = -gamepad1.left_stick_y;
            rotation = gamepad1.right_stick_x;

            // Reset scaling for number of active gamepad stick motions.
            Scale = 0;

            //Sets scale factor and applies the deadzone for translation and rotation.
            /*
            * We have a deadzone because simply, controllers aren't perfect. There are several inconsistencies
            * with the analog sticks and there will always be a very small reading because of different vibration.
            * These vibrations, without slowmode, will have a major impact on the driving experience.
             */

            if(Math.abs(translatex) < DEADZONE){
                translatex = 0;
            } else {
                Scale++;
            }
            if(Math.abs(translatey) < DEADZONE){
                translatey = 0;
            } else {
                Scale++;
            }
            if(Math.abs(rotation) < DEADZONE){
                rotation = 0;
            } else {
                Scale++;
            }

            if(Scale == 0) {
                Scale = 1;
            }

            //Computes inputs and returns an output that will become the amount of power sent to the motors.
            LeftFrontSpeed = (translatey + translatex + rotation)/Scale;
            LeftBackSpeed = (translatey - translatex + rotation)/Scale;
            RightFrontSpeed = (translatey - translatex - rotation)/Scale;
            RightBackSpeed =  (translatey + translatex - rotation)/Scale;

            //Sends power to the motors.
            LeftFront.setPower(LeftFrontSpeed);
            LeftBack.setPower(LeftBackSpeed);
            RightFront.setPower(RightFrontSpeed);
            RightBack.setPower(RightBackSpeed);

            if(gamepad2.x) {                            // Raise servo hooks to higest position
                FrontHook.setPosition(FRONT_HOOK_PARK);
                BackHook.setPosition(BACK_HOOK_PARK);
            }
            if(gamepad2.a) {                            // Lower servo hooks to foundation position
                FrontHook.setPosition(FRONT_HOOK_DOWN);
                BackHook.setPosition(BACK_HOOK_DOWN);
            }
            if(gamepad2.b) {                            // Lower servo hooks to stone position
                FrontHook.setPosition(FRONT_HOOK_STONE);
                BackHook.setPosition(BACK_HOOK_STONE);
            }

        }
        LeftBack.setPower(0);
        LeftFront.setPower(0);
        RightBack.setPower(0);
        RightFront.setPower(0);
    }
}
