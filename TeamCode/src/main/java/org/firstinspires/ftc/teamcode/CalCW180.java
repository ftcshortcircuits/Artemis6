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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Calibration tool: rotates clockwise 180 degrees by Run To Position.  Measure actual distance and adjust
 * parameters in MecanumChassis.
 */

@Autonomous(name="CalCW180", group="Encoder")
@Disabled
public class CalCW180 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();    // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private double          totalRuntime = 0;                   // Record at opMode end
    private double          cycles  = 0 ;                       // To count iterations

    private double          TIMEOUT = 30 ;                      // Max permitted runtime


    @Override
    public void runOpMode() {

        MecanumChassis drivetrain = new MecanumChassis(hardwareMap, "lFront",
                "rFront", "lBack", "rBack");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();

        waitForStart();
        runtime.reset();        // Reset the max runtime clock
        cycles = 0.0;           // Reset the iteration count
/*
 * State Machine
 *
 *  State 1:    Initialization - start rotation and set state 2
 *  State 2:    Loop until motion complete in state 2, set state 3
 */
        double speed = 0.4;         // 40% speed
        double leg_length = 180.0;  // Rotation in degrees
        int state = 1;
        while(opModeIsActive() && state < 3 && runtime.seconds() < TIMEOUT){
            cycles++;
            switch(state){
                case 1:
                    drivetrain.rotate(MecanumChassis.Direction.CLOCKWISE, leg_length, speed);
                    state = 2;
                    break;

                case 2:
                    if(!drivetrain.isBusy()){
                        state = 3;
                    }
                    break;

                case 3:
                default:
            }
            idle();
        }
        totalRuntime = runtime.milliseconds();
        drivetrain.stopDrive();
        telemetry.addData("Runtime (ms) = ", totalRuntime);
        telemetry.addData("No. of cycles = ", cycles);
        telemetry.addData("Cycle time (ms) = ", totalRuntime/cycles);
        telemetry.update();
        sleep(30000);
    }

}
