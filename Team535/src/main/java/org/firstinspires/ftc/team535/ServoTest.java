/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.team535;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Test", group = "Teleop")
//@Disabled

public class ServoTest extends OpMode {

    HardwareTOBOR robo = new HardwareTOBOR();
    double speedControl = 1;
    boolean toggleR = false;
    boolean runningR = false;
    boolean toggleL = false;
    boolean runningL = false;


    @Override
    public void init() {

        robo.initRobo(hardwareMap);
        telemetry.addData("Status:", "Robot is Initialized");

    }

    @Override
    public void init_loop() { }


    @Override
    public void loop() {

        robo.RPlate.setPosition(robo.RPlate.getPosition()+(0.002*Range.clip(1,1,gamepad1.right_stick_y)));

        robo.LPlate.setPosition(robo.LPlate.getPosition()+(0.002*Range.clip(1,1,gamepad1.left_stick_y)));

        telemetry.addData("RPlate", robo.RPlate.getPosition());
        telemetry.addData("LPlate", robo.LPlate.getPosition());
    }


    @Override
    public void stop() {
        robo.FRMotor.setPower(0);
        robo.BRMotor.setPower(0);
        robo.FLMotor.setPower(0);
        robo.BLMotor.setPower(0);
        robo.rightTrack.setPower(0);
        robo.leftTrack.setPower(0);
    }
}