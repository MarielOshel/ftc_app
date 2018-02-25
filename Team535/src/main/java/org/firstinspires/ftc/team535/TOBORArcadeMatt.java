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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math.*;

@TeleOp(name = "TOBOR Arcade Matt Onbot", group = "Teleop")
@Disabled

public class TOBORArcadeMatt extends OpMode {

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

    boolean toggleRunmode = false;
    boolean fastRunmode = true;
    Orientation angles;
    @Override
    public void init() {

        robo.initRobo(hardwareMap);
        telemetry.addData("Status:", "Robot is Initialized");
        robo.arm(HardwareTOBOR.armPos.Up);
        Orientation angles;
        angles = robo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    @Override
    public void init_loop() { }


    @Override
    public void loop() {
        telemetry.addData("Movement", "G1: Right Stick(Arcade)");
        telemetry.addData("Runmode", "G1: Right Stick Button (Fast/Slow)");
        telemetry.addData("Lower Track", "G2: Left Stick Y");
        telemetry.addData("Upper Track", "G2: Right Stick Y");
        telemetry.addData("Flippy Dipper", "G2: Hold Right Bumper");
        telemetry.addData("Jewel Arm Down", "G2: Dpad Down");
        telemetry.addData("Jewel Arm Up", "G2: Dpad Up");
        telemetry.addData("Jewel Arm Back", "G2: Dpad Right or Left");

        double leftx = Range.clip(-gamepad1.left_stick_y,-1,1);
        double rightx = -Range.clip(gamepad1.right_stick_y,-1,1);
        double righty = Range.clip(gamepad1.right_stick_x,-1,1);
        angles = robo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = (Math.atan2(righty, rightx));
        telemetry.addData("ATAN2", Math.atan2(righty, rightx));
        double otherAngle = 0;
        double hypotenuse = (Math.sqrt((rightx*rightx) + (righty * righty)))*speedControl;
        if (angle < 0)
        {
            angle = ((2 * Math.PI) + angle);
        }


        if (-angles.firstAngle <0)
        {
            otherAngle = ((360 + -angles.firstAngle)/180)*Math.PI;
            angle = angle-(otherAngle);
        }
        else
        {
            otherAngle = (-angles.firstAngle/180)*Math.PI;
            angle = angle-(otherAngle);
        }


        telemetry.addData("angle", angle);
        telemetry.addData("other angle", otherAngle);

        robo.BLMotor.setPower((hypotenuse*Math.cos(angle+(Math.PI/4)))-(leftx*speedControl));
        robo.FRMotor.setPower((hypotenuse*Math.cos(angle+(Math.PI/4)))+(leftx*speedControl));
        robo.BRMotor.setPower((hypotenuse*Math.sin(angle+(Math.PI/4)))+(leftx*speedControl));
        robo.FLMotor.setPower((hypotenuse*Math.sin(angle+(Math.PI/4)))-(leftx*speedControl));


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



            if (gamepad2.right_bumper) {
                robo.RPlate.setPosition(robo.RPlateupval);
                robo.LPlate.setPosition(robo.LPlateupval);
            } else {
                robo.RPlate.setPosition(robo.RPlatedownval);
                robo.LPlate.setPosition(robo.LPlatedownval);
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

        /*if (gamepad1.right_bumper)
        {
            robo.relicArmTurn.setPower(1);
        }
        else if (gamepad1.left_bumper)
        {
            robo.relicArmTurn.setPower(-1);
        }
        else
        {
            robo.relicArmTurn.setPower(0);
        }*/
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
