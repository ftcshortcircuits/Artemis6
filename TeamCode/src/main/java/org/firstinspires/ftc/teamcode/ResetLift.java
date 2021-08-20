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
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 *  ResetLift: Runs the motor liftMotor at a hard-coded low power level, raising it until stall is detected.
 *  ("Stall" is defined as 10 ms with no change in encoder ticks.  Stall detection does not begin until 500 ms
 *  (0.5 sec) into the cycle in order to allow the motor to start running.)
 *
 *  After stall, the lift is run in the opposite direction for LIFT_RUN_LENGTH to lower the lift, and reset
 *  the encoders to zero when the height is zero.
 */

@TeleOp(name="ResetLift", group="Linear Opmode")
// @Disabled
public class ResetLift extends LinearOpMode {

    static final double     MM_PER_INCH = 25.4;
// Physical parameters for lift powerplant
    static final double     LIFT_COUNTS_PER_MOTOR_REV   = 28.0; // Encoder countable events / rev
    static final double     LIFT_GEARBOX_REDUCTION      = 19.2; // goBILDA 5202-0002-0019
    static final double     LIFT_WINCH_DIAMETER         = (38.0/MM_PER_INCH); // goBILDA 3407-0016-0001
    static final double     LIFT_RUN_LENGTH             = 20.5; // Travel from full down to full up - inches

    static final double     LIFT_COUNTS_PER_INCH    = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_GEARBOX_REDUCTION) / (LIFT_WINCH_DIAMETER * Math.PI);
    static final double     LIFT_UP_SPEED       = 0.25;     // Rise speed for reset run.  Positive or negative TBD.
    static final double     LIFT_DOWN_SPEED     = -0.05;    // Drop speed for reset run.  Sign should not matter in RUN_TO_POSITION

    private DcMotor liftMotor;
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double elapsed, elapsedSinceTick;
    private int cycles;
    private int ticks, newTicks;
    private int retractTargetPosition;
    private boolean reported;
    private boolean running;

    @Override
    public void runOpMode() {

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setPower(0.0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        cycles = 0;
        runtime.reset();
        elapsed = elapsedSinceTick = runtime.milliseconds();
        ticks = newTicks = liftMotor.getCurrentPosition();      // Should be zero, here
        liftMotor.setPower(LIFT_UP_SPEED);                      // Motion begins
        running = true;
        while (opModeIsActive()) {

            while (running && opModeIsActive()) {           // "running" only controls lift rise and stall testing.  For the
                                                            // descent, running will be false, but the opMode continues.
                cycles++;
                if (runtime.milliseconds() > 1000) {     // Track for stall only after first half second

                    newTicks = liftMotor.getCurrentPosition();
                    elapsed = runtime.milliseconds();

                    if(newTicks != ticks){              // We're not yet stalled
                        elapsedSinceTick = elapsed;
                        ticks = newTicks;
                    } else if( elapsed-elapsedSinceTick > 50.0) {       // STALLED !!
                        liftMotor.setPower(0.0);                        // Stop the motor
                        telemetry.addData("Ticks at top = ", "%d, in %f ms", newTicks, elapsed);
                        if(newTicks > 0){
                            telemetry.addData("Encoder is POSITIVE going on rise", "");
                            retractTargetPosition = newTicks - (int) (LIFT_COUNTS_PER_INCH * LIFT_RUN_LENGTH);
                        } else {
                            telemetry.addData("Encoder is NEGATIVE going on rise", "");
                            retractTargetPosition = newTicks + (int) (LIFT_COUNTS_PER_INCH * LIFT_RUN_LENGTH);
                        }
                        liftMotor.setTargetPosition(retractTargetPosition);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(LIFT_DOWN_SPEED);
                        running = false;        // Breaks out of the loop and reports telemetry while retracting
                    }
            }
         }
            telemetry.update();     // Display will persist until opMode is stopped
        }
        liftMotor.setPower(0.0);
        }
    }