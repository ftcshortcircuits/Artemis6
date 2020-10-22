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
 * Derived from AutonModel, and PushbotAutoDriveByEncoder_Linear, in the external samples folder.
 */

@Autonomous(name="BLUE_LOAD", group="Encoder")
//@Disabled
public class BlueLoad extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();    // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private double          totalRuntime = 0;                   // Record at opMode end
    private double          cycles  = 0 ;                       // To count iterations

    private int             stone ;                             // First Skystone position
    private int             motionFinalState;                   // End state for GenericLoad motion
    private int             motionCurrentState;                 // Where GenericLoad is now

    private double          AUTON_TIMEOUT   = 30 ;              // Max permitted runtime
    private int             SAMPLE_COUNT    = 100000;           // Number of trials
    private double          VISION_TIMEOUT  = 5 ;               // How long we wait for stone ID


    @Override
    public void runOpMode() {

        /*
         * Instantiate a GenericLoad object - which will instantiate a drivetrain and mechanism object.
         */
        GenericLoad motion = new GenericLoad(hardwareMap, telemetry, GenericLoad.Alliance.BLUE);

        /*
         * Instantiate a Detect object - which will instantiate a Skystone detection object.
         */
        SkystoneDetector detector = new SkystoneDetector(hardwareMap, telemetry, SkystoneDetector.Alliance.BLUE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "BLUE Initialization Complete");
        telemetry.update();

        waitForStart();
        runtime.reset();        // Reset the max runtime clock
        cycles = 0.0;           // Reset the iteration count
/*
 * Blue Loading Zone autonomous skeleton.  Uses a simple state machine to loop through calls to vision
 * methods, then motion methods.
 *
 *  State 1:    Poll the skystoneDetection() method in the Detect class until loop count expires or timeout.
 *  State 2:    Retrieve the "best guess" via getDecision(); initialize and start motion.
 *  State 3:    Keep calling cycle() method in GenericLoad class.
 *  State 4:    Motion complete; immediate exit.
 */
        int state = 2;
        int visionSamples = 0;
        while(opModeIsActive() && state < 4 && runtime.seconds() < AUTON_TIMEOUT){
            cycles++;
            switch(state){
                case 1:
                    if((visionSamples<SAMPLE_COUNT) && (runtime.seconds()<VISION_TIMEOUT)) {
//                        detector.skystoneDetection();
                        visionSamples++;
                    } else {
                        state = 2;
                    }
                    break;

                case 2:
                    // detector.getStone() finds the stone position
                    stone = detector.getStone();

                    // telemetry the stone number for debugging purposes
                    telemetry.addData("Skystone = ", stone);

                    // stop the skystone detection
                    detector.stopDetect();

                    // start the motion
                    motionCurrentState = 0;
                    motionFinalState = motion.getEndState(stone);  // So we know when to stop
                    telemetry.addData("motion final state", motionFinalState);
                    telemetry.update();
//                    sleep(30000);
                    motionCurrentState = motion.cycle(motionCurrentState, stone);   // Starts the motion
                    state = 3;
//                        state = 4;
                    break;

                case 3:
                    // motion.cycle() runs the state machine and returns the new state
                    motionCurrentState = motion.cycle(motionCurrentState, stone);

                    // if the current state is greater than the last state, we know the auton movement is finished
                    if(motionCurrentState > motionFinalState){
                        state = 4;
                    }
                    break;
                case 4:
                default:
                    break;
            }
            idle();
        }
        totalRuntime = runtime.milliseconds();
//        drivetrain.stopDrive();
/*
        telemetry.addData("Runtime (ms) = ", totalRuntime);
        telemetry.addData("No. of cycles = ", cycles);
        telemetry.addData("Cycle time (ms) = ", totalRuntime/cycles);
        telemetry.update();
*/
        sleep(30000);
    }

}
