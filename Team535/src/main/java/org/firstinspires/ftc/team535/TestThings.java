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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test Things Onbot", group = "Teleop")
//@Disabled

public class TestThings extends OpMode {

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
        robo.plate(HardwareTOBOR.platePos.Down);
        robo.wrist(HardwareTOBOR.wristPos.Open);


    }

    @Override
    public void init_loop() { }


    @Override
    public void loop() {

        robo.Claw.setPosition(robo.Claw.getPosition()+(0.002*Range.clip(1,1,gamepad2.right_trigger)));
        robo.Claw.setPosition(robo.Claw.getPosition()-(0.002*Range.clip(1,1,gamepad2.left_trigger)));
        telemetry.addData("Claw", robo.Claw.getPosition());

        telemetry.addData("left Stick", Range.clip(-gamepad2.left_stick_y,-1,1));
        telemetry.addData("right Stick", Range.clip(-gamepad2.right_stick_y,-1,1));
        robo.relicArmExtend.setPower(Range.clip(-gamepad2.left_stick_y,-1,1));
        robo.relicArmWrist.setPower(Range.clip(-gamepad2.right_stick_y,-1,1)+0.1);


        robo.JArm.setPosition(robo.JArm.getPosition()+(0.002*Range.clip(1,1,gamepad1.right_trigger)));
        robo.JArm.setPosition(robo.JArm.getPosition()-(0.002*Range.clip(1,1,gamepad1.left_trigger)));
        telemetry.addData("RPlate", robo.RPlate.getPosition());
        telemetry.addData("LPlate", robo.LPlate.getPosition());
        telemetry.addData("Arm", robo.JArm.getPosition());
        if (gamepad1.a)
        {
            robo.JWrist.setPosition(robo.JWrist.getPosition()+0.002);
        }
        else if (gamepad1.b)
        {
            robo.JWrist.setPosition((robo.JWrist.getPosition() - 0.002));
        }

        telemetry.addData("Wrist", robo.JWrist.getPosition());
        robo.LPlate.setPosition(robo.LPlate.getPosition() - (0.001 * gamepad1.left_stick_y));
        robo.RPlate.setPosition(robo.RPlate.getPosition() + (0.001 * gamepad1.right_stick_y));
        if (gamepad1.left_bumper)
        {
            //.0696
            //.99938
            robo.rightTrackUp.setPower(0.5);
            robo.leftTrackUp.setPower(0);
            //18.06
            //.903 s/rev
            // adjust .83533
        }
        else if (gamepad1.right_bumper)
        {
            //1.081 s/rev
            // adjust 1
            robo.leftTrackUp.setPower(0.5);
            robo.rightTrackUp.setPower(0);
        }
        else
        {
            robo.leftTrackUp.setPower(0);
            robo.rightTrackUp.setPower(0);
        }
        telemetry.addData("Range",robo.rangeSensor.getDistance(DistanceUnit.INCH));

        /*if (gamepad1.dpad_up)
        {
            robo.relicArmTurn.setPower(1);
        }
        else if (gamepad1.dpad_down)
        {
            robo.relicArmTurn.setPower(-1);
        }
        else
        {
            robo.relicArmTurn.setPower(0);
        }

        telemetry.addData("Dpad Right", gamepad1.dpad_right);
        telemetry.addData("Dpad Left", gamepad1.dpad_left);

        if (gamepad1.dpad_right)
        {
            robo.relicArmExtend.setPower(1);
        }
        else if (gamepad1.dpad_left)
        {
            robo.relicArmExtend.setPower(-1);
        }
        else
        {
            robo.relicArmExtend.setPower(0);
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
        robo.leftTrackDown.setPower(0);
        robo.rightTrackDown.setPower(0);
        //robo.relicArmExtend.setPower(0);
        //robo.relicArmTurn.setPower(0);
    }
}
