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

@TeleOp(name="ParabolicDrive", group="Linear Opmode")
// @Disabled
public class ParabolicDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Only for auto-align
    private MecanumChassis drivetrain;
    boolean autoBusy = false;

    //auto-align stack
    private boolean autoStack = false;

    //Declaring our motors and intake.
    private DcMotor LeftBack;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor RightIntake;
    private DcMotor LeftIntake;
    private DcMotor Lift;
    private Servo FrontHook;
    private Servo BackHook;
    private Servo RightIntakeServo;
    private Servo LeftIntakeServo;
    private Servo Grabber;
    private Servo Swing;
//    private Servo CapstoneServo;
//    private CRServo Stage2Left;
//    private CRServo Stage2Right;

    //Declared variables for speed.
    private double LeftBackSpeed;
    private double RightBackSpeed;
    private double LeftFrontSpeed;
    private double RightFrontSpeed;

    //Declared variables for translation, rotation, and scale factor.
    private double translatex;
    private double translatey;
    private double rotation;
    private double spinPower;
    private double liftPower;
    private double liftPos;
    public int Scale;

    // variables for arm and grabber
    private boolean rUp = true;
    private boolean lUp = true;
    private boolean yUp = true;
    private int swingState = 0;
    private int grabberState = 0;
//    private int capstoneState = 0;

    /*
     * Adjustable parameters for the robot.  Add new mechanism-related constants here.
     */

    //A lower number means a higher position for the hooks
    private double FRONT_HOOK_PARK          = 0.17  ;        // Park location (full up) for servo hook
    private double FRONT_HOOK_DOWN          = 0.65  ;        // Foundation drag position for servo hook
    private double FRONT_HOOK_STONE         = 0.57  ;        // Stone drag position for servo hook

    //A higher number means a higher position for the hooks
    private double BACK_HOOK_PARK           = 0.91  ;        // Park location (full up) for servo hook
    private double BACK_HOOK_DOWN           = 0.35  ;        // Foundation drag position for servo hook
    private double BACK_HOOK_STONE          = 0.45  ;        // Stone drag position for servo hook


    //A higher number means closer to the ground near stone position
    private double RIGHT_INTAKE_SERVO_PARK  = 0.16  ;        // Park location (full up) for the intake servo
    private double RIGHT_INTAKE_SERVO_MID1  = 0.45  ;        // Middle position for a action yet to be determined.
    private double RIGHT_INTAKE_SERVO_MID2  = 0.35  ;        // Middle position for a action yet to be determined.
    private double RIGHT_INTAKE_SERVO_STONE = 0.47  ;        // Stone pickup location for the intake servo

    //A lower number means closer to the ground near stone position
    private double LEFT_INTAKE_SERVO_PARK   = 0.87  ;        // Park location (full up) for the intake servo
    private double LEFT_INTAKE_SERVO_MID1   = 0.70  ;        // Middle position for a action yet to be determined.
    private double LEFT_INTAKE_SERVO_MID2   = 0.60  ;        // Middle position for a action yet to be determined.
    private double LEFT_INTAKE_SERVO_STONE  = 0.57 ;        // Stone pickup location for the intake servo

    //0.3 was the original speed and it was medium-fast.
    private double LIFT_SPEED               = 0.45  ;        // The max speed of the lift

    //A higher number means a harder clamp (some positions will be out of range)
    private double GRABBER_PARK             = 0.20  ;        // The position where the grabber doesn't hold the stone
    private double GRABBER_STONE            = 0.55  ;        // The position where the grabber holds the stone

    //A lower number means that the grabber rotates clockwise because the servo is upside down.
    //The servo will actually be rotating counterclockwise with a lower number.
    private double SWING_PARK               = 0.5445;        // The position where the swing is pointing to the front
    private double SWING_STACK              = 0.48  ;        // The position where the swing is pointing backwards

    //
    private double CAPSTONE_SERVO_PARK      = 0.0   ;        // The position where the capstone deployment system is pointing backwards
    private double CAPSTONE_SERVO_DEPLOY    = 1.0   ;        // The position where the capstone deployment system is deploying the capstone and is pointing forwards

    static final double     MM_PER_INCH = 25.4;

    // Physical parameters for lift powerplant
    static final double     LIFT_COUNTS_PER_MOTOR_REV   = 28.0; // Encoder countable events / rev
    static final double     LIFT_GEARBOX_REDUCTION      = 19.2; // goBILDA 5202-0002-0019
    static final double     LIFT_WINCH_DIAMETER         = (38.0/MM_PER_INCH); // goBILDA 3407-0016-0001
    static final double     LIFT_RUN_LENGTH             = 20.5; // Travel from full down to full up - inches
    static final double     LIFT_COUNTS_PER_INCH        = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_GEARBOX_REDUCTION) / (LIFT_WINCH_DIAMETER * Math.PI);
    static final int        MIN_HEIGHT                  = 0;
    static final int        MAX_HEIGHT                  = (int) (LIFT_COUNTS_PER_INCH * LIFT_RUN_LENGTH);

    private double DEADZONE                 = 0.05 ;        // Center deadzone for gamepad sticks and spinPower

    @Override
    public void runOpMode() {
        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initializing motors and intake.
        LeftBack  = hardwareMap.get(DcMotor.class, "lBack");
        RightBack  = hardwareMap.get(DcMotor.class, "rBack");
        LeftFront  = hardwareMap.get(DcMotor.class, "lFront");
        RightFront  = hardwareMap.get(DcMotor.class, "rFront");
        LeftIntake = hardwareMap.get(DcMotor.class, "rIntakeM");
        RightIntake = hardwareMap.get(DcMotor.class, "lIntakeM");
        Lift = hardwareMap.get(DcMotor.class, "liftMotor");
        FrontHook =  hardwareMap.get(Servo.class,  "fHook");
        BackHook = hardwareMap.get(Servo.class, "bHook");
        RightIntakeServo = hardwareMap.get(Servo.class, "rIntakeS");
        LeftIntakeServo = hardwareMap.get(Servo.class, "lIntakeS");
        Grabber = hardwareMap.get(Servo.class, "grabServo");
        Swing = hardwareMap.get(Servo.class, "swingServo");
//        CapstoneServo = hardwareMap.get(Servo.class, "capstoneServo")
//        Stage2Left = hardwareMap.get(CRServo.class, "s2l");
//        Stage2Right = hardwareMap.get(CRServo.class, "s2r");

        //Only for auto-align
        drivetrain = new MecanumChassis(hardwareMap, "lFront","rFront", "lBack", "rBack");

        // set the modes of all the drive train motors to RUN_USING_ENCODERS
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reverses the powers of the right motors
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        // resets the encoder ticks for the lift
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // sets the mode of the lift motor to RUN_USING_ENCODER
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        Stage2Right.setDirection(CRServoImpl.Direction.REVERSE);

        // stops all of the motors
        LeftBack.setPower(0);
        LeftFront.setPower(0);
        RightBack.setPower(0);
        RightFront.setPower(0);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
//        Stage2Left.setPower(0);
//        Stage2Right.setPower(0);

        // sets all of the servo positions to initialization positions
        FrontHook.setPosition(FRONT_HOOK_PARK);
        BackHook.setPosition(BACK_HOOK_PARK);
        RightIntakeServo.setPosition(RIGHT_INTAKE_SERVO_STONE);
        LeftIntakeServo.setPosition(LEFT_INTAKE_SERVO_STONE);
        Grabber.setPosition(GRABBER_PARK);
        Swing.setPosition(SWING_PARK);
//        CapstoneServo.setPosition(CAPSTONE_SERVO_PARK);

        // sets the zero power behavior for the motors

        // drive train motors
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // intake motors
        LeftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // lift motor
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            spinPower = -gamepad2.right_stick_y;

            // this it to find the rUp and lUp vars
            if (!gamepad2.right_bumper) {
                rUp = true;
            }
            if (!gamepad2.left_bumper) {
                lUp = true;
            }
            // Reset scaling for number of active gamepad stick motions.
            Scale = 0;

            //Sets scale factor and applies the deadzone for translation and rotation.
            /*
             * We have a deadzone because simply, controllers aren't perfect. There are several inconsistencies
             * with the analog sticks and there will always be a very small reading because of different vibration.
             * These vibrations, without slowmode, will have a major impact on the driving experience.
             */

            if (Math.abs(translatex) < DEADZONE) {
                translatex = 0;
            } else {
                Scale++;
            }
            if (Math.abs(translatey) < DEADZONE) {
                translatey = 0;
            } else {
                Scale++;
            }
            if (Math.abs(rotation) < DEADZONE) {
                rotation = 0;
            } else {
                Scale++;
            }
            if (Math.abs(spinPower) < DEADZONE) {
                spinPower = 0;
            }
            if (Scale == 0) {
                Scale = 1;
            }

            /*
             * Shape the gamepad stick (throttle) response to a parabolic curve for better control at
             * low speed.  EXPERIMENTAL.
             */
            translatex = Math.copySign(translatex * translatex, translatex);
            translatey = Math.copySign(translatey * translatey, translatey);
            rotation = Math.copySign(rotation * rotation, rotation);

            //Computes inputs and returns an output that will become the amount of power sent to the motors.
            LeftFrontSpeed = (translatey + translatex + rotation) / Scale;
            LeftBackSpeed = (translatey - translatex + rotation) / Scale;
            RightFrontSpeed = (translatey - translatex - rotation) / Scale;
            RightBackSpeed = (translatey + translatex - rotation) / Scale;

            //this checks if we should call isBusy()
            if (autoBusy) {
                autoBusy = drivetrain.isBusy();
                /*
                 * if the drivetrain is finished then it will return false so we check if it changes
                 * from true to false and that means the alignment has finished.
                 */
                if (!autoBusy) {
                    LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    FrontHook.setPosition(FRONT_HOOK_DOWN);
                    BackHook.setPosition(BACK_HOOK_DOWN);
                    autoBusy = false;
                }
            }else if(autoStack){
                autoStack = drivetrain.isBusy();

                if(!autoStack){
                    LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }else {
                //Sends power to the motors, both the intake motors, and the CRServos.
                LeftFront.setPower(LeftFrontSpeed);
                LeftBack.setPower(LeftBackSpeed);
                RightFront.setPower(RightFrontSpeed);
                RightBack.setPower(RightBackSpeed);
                RightIntake.setPower(spinPower);
                LeftIntake.setPower(spinPower);
            }

/*            if(spinPower != 0) {
                Stage2Left.setPower(Math.copySign(1, spinPower));
                Stage2Right.setPower(Math.copySign(1, spinPower));
            }else{
                Stage2Left.setPower(0);
                Stage2Right.setPower(0);
            }*/
            if(gamepad2.x) {                            // Raise servo hooks to highest position
                FrontHook.setPosition(FRONT_HOOK_PARK);
                BackHook.setPosition(BACK_HOOK_PARK);
            }
            if(gamepad2.a && !autoBusy) {                            // Lower servo hooks to foundation position
                drivetrain.strafeDrive(MecanumChassis.Direction.RIGHT, 3, 0.15);
                // This will trigger the "if (autoBusy)" statement
                autoBusy = true;
            }
            if(gamepad2.b) {                            // Lower servo hooks to stone position
                FrontHook.setPosition(FRONT_HOOK_STONE);
                BackHook.setPosition(BACK_HOOK_STONE);
            }
            if(gamepad2.dpad_up){                       // Raise intake servo to park position
                RightIntakeServo.setPosition(RIGHT_INTAKE_SERVO_PARK);
                LeftIntakeServo.setPosition(LEFT_INTAKE_SERVO_PARK);
            }
            if(gamepad2.dpad_down){                     // Lower intake servo to stone position
                RightIntakeServo.setPosition(RIGHT_INTAKE_SERVO_STONE);
                LeftIntakeServo.setPosition(LEFT_INTAKE_SERVO_STONE);
            }
            if(gamepad2.dpad_left){                       // Raise or lower intake servo to intermediate position 1
                RightIntakeServo.setPosition(RIGHT_INTAKE_SERVO_MID1);
                LeftIntakeServo.setPosition(LEFT_INTAKE_SERVO_MID1);
            }
            if(gamepad2.dpad_right) {                     // Raise or lower intake servo to intermediate position 2
                RightIntakeServo.setPosition(RIGHT_INTAKE_SERVO_MID2);
                LeftIntakeServo.setPosition(LEFT_INTAKE_SERVO_MID2);
            }

            // setting variables for conditional to stop the motor
            liftPower = -gamepad2.left_stick_y;
            liftPos = Lift.getCurrentPosition();

            telemetry.addData("encoder ticks = ", liftPos);
            telemetry.update();

            // Due to joystick problems we have a deadzone
            if(Math.abs(liftPower) < DEADZONE){
                liftPower = 0;
            }
            // Since we are using a lift we do not want to spin the motor above the lift's limits.
            /*
            * If the position of the lift is higher than max height and the operator is trying to go
            * higher, then the program will not allow that.
            */
            if(liftPos>=MAX_HEIGHT && liftPower > 0){
                liftPower = 0;
            }

            /*
            * If the position of the lift is lower than the min height and the operator is trying to
            * go lower, then the program will not allow that.
            */
            if(liftPos <= MIN_HEIGHT && liftPower < 0){
                liftPower = 0;
            }

            Lift.setPower(liftPower * LIFT_SPEED);

            // if the bumpers are up in the previous cycle then we can move it
            if(lUp && gamepad2.left_bumper){
                // if the grabber is in park position and the left bumper is pressed, then the grabber will switch to stone position
                lUp = false;
                if(grabberState == 0){
                    grabberState = 1;
                    Grabber.setPosition(GRABBER_STONE);
                }
                // if the grabber is in stone position and the left bumper is pressed, then the grabber will switch to park position
                else{
                    grabberState = 0;
                    Grabber.setPosition(GRABBER_PARK);
                }
            }
            // if the bumpers are up in the previous cycle then we can move it
            if(rUp && gamepad2.right_bumper) {
                // if the swing is in park position and the right bumper is pressed, then the swing will switch to the stack position
                rUp = false;
                if (swingState == 0) {
                    swingState = 1;
                    Swing.setPosition(SWING_STACK);
                }
                // if the swing is in the stack position and the right bumper is pressed, then the swing will switch to park position
                else {
                    swingState = 0;
                    Swing.setPosition(SWING_PARK);
                }
            }
            if(gamepad1.b && !autoStack){
                autoStack = true;
                //if we trigger the autoStack then we want to strafe right for auto stacking purposes
                drivetrain.longDrive(MecanumChassis.Direction.FORWARD, 4.5, 0.15);
            }
            /*
            // if the y button was up and now its down, then we want to change the position(this stops the problem of holding the button)
            if(yUp && gamepad2.y){
                // since we know that the y button is down, we set yUp to false
                yUp = false;
                if(capstoneState == 0){
                    // if we are in the deploy state then we want to park
                    capstoneState = 1;
                    CapstoneServo.setPosition(CAPSTONE_SERVO_PARK);
                }
                else {
                    // if we are in the park state then we want to deploy
                    capstoneState = 0;
                    CapstoneServo.setPosition(CAPSTONE_SERVO_DEPLOY);
                }
            }
            */

        }
        //Sets power to zero at the end of the opmode.
        LeftBack.setPower(0);
        LeftFront.setPower(0);
        RightBack.setPower(0);
        RightFront.setPower(0);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
        Lift.setPower(0);
//        Stage2Left.setPower(0);
//        Stage2Right.setPower(0);
    }
}
