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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TOBOR Tank Drive", group = "Teleop")
@Disabled

public class TOBORTank extends OpMode {

    HardwareTOBOR robo = new HardwareTOBOR();
    double speedControl;
    boolean toggleRB = false;
    boolean runningRB = false;
    boolean toggleLB = false;
    boolean runningLB = false;

    boolean toggleRT = false;
    boolean runningRT = false;
    boolean toggleLT = false;
    boolean runningLT = false;
//poop
    boolean toggleRunmode = false;
    boolean fastRunmode = true;
    @Override
    public void init() {

        robo.initRobo(hardwareMap);
        telemetry.addData("Status:", "Robot is Initialized");
        robo.arm(HardwareTOBOR.armPos.Up);
    }

    @Override
    public void init_loop() { }


    @Override
    public void loop() {

        if (gamepad1.right_trigger >= 0.1) {
            robo.strafeRight(gamepad1.right_trigger*speedControl);
        } else if (gamepad1.left_trigger >= 0.1) {
            robo.strafeLeft(gamepad1.left_trigger*speedControl);
        } else {
            robo.FRMotor.setPower(Range.clip(-gamepad1.left_stick_y*speedControl, -1, 1));
            robo.BRMotor.setPower(Range.clip(-gamepad1.left_stick_y*speedControl, -1, 1));
            robo.FLMotor.setPower(Range.clip(-gamepad1.right_stick_y*speedControl, -1, 1));
            robo.BLMotor.setPower(Range.clip(-gamepad1.right_stick_y*speedControl, -1, 1));
        }


        if (toggleRunmode)
        {
            if (gamepad1.right_stick_button)
            {
                fastRunmode = !fastRunmode;
                toggleRunmode = false;
            }
        }
        else if (!(gamepad1.right_stick_button))
        {
            toggleRunmode = true;
        }


        if (fastRunmode)
        {
            speedControl = 1;
        }
        else if (!fastRunmode)
        {
            speedControl = 0.5;
        }

        robo.rightTrackDown.setPower(-Range.clip(gamepad2.left_stick_y, -1, 1));
        robo.leftTrackDown.setPower(-Range.clip(gamepad2.left_stick_y, -1, 1));
        robo.rightTrackUp.setPower(.83533 * (-Range.clip(gamepad2.right_stick_y,-1,1)));
        robo.leftTrackUp.setPower(-Range.clip(gamepad2.right_stick_y, -1,1));

        
        
        if (gamepad2.right_bumper)
        {
            robo.RPlate.setPosition(.08);
            robo.LPlate.setPosition(1);
        }
        else
        {
            robo.RPlate.setPosition(.81);
            robo.LPlate.setPosition(.27);
        }


        if (gamepad2.dpad_up)
        {
            robo.arm(HardwareTOBOR.armPos.Up);
        }
        else if (gamepad2.dpad_down)
        {
            robo.arm(HardwareTOBOR.armPos.Down);
        }
        else if (gamepad2.dpad_left||gamepad2.dpad_right)
        {
            robo.arm(HardwareTOBOR.armPos.Back);
        }
        /*if (gamepad2.right_trigger >= 0.1)
        {
            robo.relicArmTurn.setPower(1);
        }
        else if (gamepad2.left_trigger >=0.1)
        {
            robo.relicArmTurn.setPower(-1);
        }
        else
        {
            robo.relicArmTurn.setPower(0);
        }*/
        telemetry.addData("RPlate", robo.RPlate.getPosition());
        telemetry.addData("LPlate", robo.LPlate.getPosition());
        telemetry.addData("JoystickL", gamepad2.left_stick_y);
        telemetry.addData("JoystickR", gamepad2.right_stick_y);
    }


    @Override
    public void stop() {
        robo.FRMotor.setPower(0);
        robo.BRMotor.setPower(0);
        robo.FLMotor.setPower(0);
        robo.BLMotor.setPower(0);
        robo.rightTrackUp.setPower(0);
        robo.leftTrackUp.setPower(0);
        robo.rightTrackDown.setPower(0);
        robo.leftTrackDown.setPower(0);
        robo.RPlate.setPosition(.81);
        robo.LPlate.setPosition(.27);

    }
}