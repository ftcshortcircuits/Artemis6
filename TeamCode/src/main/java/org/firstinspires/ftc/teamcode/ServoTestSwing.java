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
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Tests a single servo.  Name is hard-coded, and will have to be edited to match configuration.  Servo
 * will move to zero at "Run", then respond with movement on each button press: "Y" to increase servo
 * position, "A" to reduce it.
 */

@TeleOp(name="ServoTestSwing", group="Linear Opmode")
@Disabled
public class ServoTestSwing extends LinearOpMode {

    private Servo testServo;

    private boolean yUp, aUp;

    private double testPos = 0.50;
    private double step = 0.01;

    @Override
    public void runOpMode() {

        testServo = hardwareMap.get(Servo.class, "swingServo");  // Hardcoded configuration name

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Initial position will be ", testPos);
        telemetry.addData("Button 'Y' increases, 'A' decreases, in steps of ", step);
        telemetry.update();

        waitForStart();

        testServo.setPosition(testPos);     // Initial position
        aUp = true;
        yUp = true;

        while (opModeIsActive()){

            if(!gamepad2.a)
                aUp = true;
             if(!gamepad2.y)
                yUp = true;

            if(gamepad2.a && aUp){
                aUp = false;
                testPos -= step;
                testServo.setPosition(testPos);
                telemetry.addData("Servo position: ","%.03f", testPos);
                telemetry.update();
            }
            if(gamepad2.y && yUp){
                yUp = false;
                testPos += step;
                testServo.setPosition(testPos);
                telemetry.addData("Servo position: ","%.03f", testPos);
                telemetry.update();
            }

        }
    }
}
