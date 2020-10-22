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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Derived from PushbotAutoDriveByEncoder_Linear, in the external samples folder.
 * Use this to park on the alliance skybridge tape.
 */

@Autonomous(name="PARK - blue_build", group="Encoder")
//@Disabled
public class ParkBlueBuild extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        MecanumChassis drivetrain = new MecanumChassis(hardwareMap, "lFront",
                "rFront", "lBack", "rBack");
        Mechanisms mechanisms = new Mechanisms(hardwareMap, "fHook", "bHook", "rIntakeS", "lIntakeS");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();

        waitForStart();
/*
 * Autonomous program to park on the alliance skybridge tape, waiting for each movement
 * to complete before moving on.  Using a state machine.
 *
 *  State 1:    Initialization - start going forward 2 inches and set state to 2 and break
 *  State 2:    Check if busy - start strafing right 39 inches and set state to 3 to exit Opmode
 *
 */
        double speed = 0.4;         // 40% speed
        int state = 1;
        sleep(20000);
        while(opModeIsActive() && state < 5){
            switch(state){
                case 1:
                    // go forward to get off the field perimeter
                    drivetrain.strafeDrive(MecanumChassis.Direction.LEFT, 4, speed);
                    state = 2;
                    break;
                case 2:
                    // strafe right to land on the alliance skybridge tape
                    if(!drivetrain.isBusy()){
                        drivetrain.longDrive(MecanumChassis.Direction.FORWARD, 32.0, speed);
                        state = 3;
                    }
                    break;
                case 3:
                    if(!drivetrain.isBusy()){
                        mechanisms.intakeDown();
                        runtime.reset();
                        state = 4;
                    }
                    break;
                case 4:
                    if(runtime.milliseconds() > 5000) {
                        state = 5;
                    }
                    break;

                case 5:
                default:
            }
            idle();
        }
        drivetrain.stopDrive();

    }

}
