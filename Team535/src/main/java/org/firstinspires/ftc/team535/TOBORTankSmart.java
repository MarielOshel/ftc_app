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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TOBOR Tank Smart Onbot", group = "Teleop")
//@Disabled

public class TOBORTankSmart extends OpMode {

    HardwareTOBOR robo = new HardwareTOBOR();
    double speedControl;
    boolean toggleRB = false;
    boolean runningRB = false;
    boolean toggleLB = false;
    boolean runningLB = false;

    boolean flicking;
    float offset = 0;

    boolean toggleRT = false;
    boolean runningRT = false;
    boolean toggleLT = false;
    boolean runningLT = false;

    boolean toggleRev = true;
    boolean goRev = false;

    boolean toggleRunmode = false;
    boolean fastRunmode = true;

    boolean toggleBrake = false;
    boolean isBrake = false;
    
    double gameyleft;
    double gameyright;
    double gamelefttrig;
    double gamerighttrig;
    
    double gameyavg;
    double adjustment;
    
    boolean toggleGyroMode = true;
    boolean gyroMode = false;

    boolean toggleClaw = false;
    boolean clawOpen = false;

    double extend;
    double wrist;
    Orientation angles;

    @Override
    public void init() {

        robo.initRobo(hardwareMap);
        telemetry.addData("Status:", "Robot is Initialized");
        robo.arm(HardwareTOBOR.armPos.Back);
        robo.rightTrackDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robo.rightTrackUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robo.leftTrackDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robo.leftTrackUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);





    }

    @Override
    public void init_loop() {
    }


    @Override
    public void loop() {
        angles = ((robo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)));
        if (gamepad2.start)
        {
            flicking = true;

        }
        if (flicking) {
            robo.BRMotor.setPower(0);
            robo.BLMotor.setPower(0);
            robo.FRMotor.setPower(0);
            robo.FLMotor.setPower(0);
            robo.rightTrackDown.setPower(0);
            robo.leftTrackDown.setPower(0);
            robo.rightTrackUp.setPower(0);
            robo.leftTrackUp.setPower(0);
            if (robo.runtime.seconds() <= 0.5) {
                robo.arm(HardwareTOBOR.armPos.Down);
                robo.wrist(HardwareTOBOR.wristPos.Open);
            } else if (robo.runtime.seconds() >= 0.5 && robo.runtime.seconds() <= 0.8) {
                robo.wrist(HardwareTOBOR.wristPos.Right);
            } else if (robo.runtime.seconds() >= 0.8 && robo.runtime.seconds() <= 1.1) {
                robo.wrist(HardwareTOBOR.wristPos.Left);
            } else if (robo.runtime.seconds() >= 1.1) {
                flicking = false;
            }
            if (!flicking) {
                robo.arm(HardwareTOBOR.armPos.Back);
                robo.runtime.reset();
                robo.wrist(HardwareTOBOR.wristPos.Closed);
            }
        }
        if (gamepad1.left_bumper&&gamepad1.right_bumper&&gamepad1.left_stick_button)
        {
            offset = angles.firstAngle;
        }

        if (toggleRev) {
            if (((angles.firstAngle-offset) >= 105 || (angles.firstAngle-offset) <= -105)  && Math.abs(gamepad1.left_stick_y) <.2 && Math.abs(gamepad1.right_stick_y) <.2&& Math.abs(gamepad1.left_trigger) <.1&& Math.abs(gamepad1.right_trigger) <.2) {
                goRev = true;
                toggleRev = false;
            }
        } else if (!((angles.firstAngle-offset) >= 105 || (angles.firstAngle-offset) <= -105)  && Math.abs(gamepad1.left_stick_y) <.2 && Math.abs(gamepad1.right_stick_y) <.2&& Math.abs(gamepad1.left_trigger) <.1&& Math.abs(gamepad1.right_trigger) <.2) {
            toggleRev = true;
            goRev = false;
        }
        
        if (toggleGyroMode)
        {
            if (gamepad1.b)
            {
                gyroMode = !gyroMode;
                toggleGyroMode = false;
            }
        }
        else if (!toggleGyroMode && !gamepad1.b)
        {
            toggleGyroMode = true;
        }

        if (toggleClaw)
        {
            if (gamepad1.y)
            {
                clawOpen = !clawOpen;
                toggleClaw = false;
            }
        }
        else if (!toggleClaw && !gamepad1.y)
        {
            toggleClaw = true;
        }


        if (gamepad1.y) {
            robo.Claw.setPosition(robo.Claw.getPosition() +0.008);
        }
        else if (clawOpen)
        {
            robo.claw(HardwareTOBOR.clawPos.Open);
        }
        else if (!clawOpen) {
            robo.claw(HardwareTOBOR.clawPos.Closed);
        }


        telemetry.addData("toggleRev", toggleRev);
        telemetry.addData("angles", (angles.firstAngle-offset));



        telemetry.addData("Gorev", goRev);
        if (!goRev) {
            gameyleft = Range.clip(gamepad1.left_stick_y, -1, 1);
            gameyright = Range.clip(gamepad1.right_stick_y, -1, 1);
            gamelefttrig = gamepad1.left_trigger;
            gamerighttrig = gamepad1.right_trigger;
        }
         else{ 

            gameyleft = Range.clip(gamepad1.left_stick_y, -1, 1);
            gameyright = Range.clip(gamepad1.right_stick_y, -1, 1);
            gamelefttrig = gamepad1.right_trigger;
            gamerighttrig = gamepad1.left_trigger;

            }
        if (gamepad1.x)
        {
            extend = 0.9;
            wrist = 1;

        }
        else if (gamepad1.dpad_right||gamepad1.dpad_left)
        {
            extend = -0.9;
            wrist = -1;
        }
        else
        {
            extend = 0;
            wrist = 0;
        }

        if (gamepad1.dpad_up)
        {
            wrist = Range.clip(wrist+1,-1,1);
        }
        else if (gamepad1.dpad_down)
        {
            wrist = Range.clip(wrist-1,-1,1);
        }
        robo.relicArmExtend.setPower(extend);
        robo.relicArmWrist.setPower(wrist);

            
            if (gyroMode)
            {
                gameyavg = (gameyleft + gameyright)/2;
                adjustment = (angles.firstAngle-offset)/40;
                if (Math.abs((angles.firstAngle-offset))<=2)
                {
                    adjustment = 0;
                }
                robo.BLMotor.setPower(Range.clip(-gameyavg - adjustment-gamerighttrig + gamelefttrig,-1,1));
                robo.BRMotor.setPower(Range.clip(-gameyavg + adjustment + gamerighttrig - gamelefttrig,-1,1));
                robo.FRMotor.setPower(Range.clip(-gameyavg + adjustment-gamerighttrig + gamelefttrig,-1,1));
                robo.FLMotor.setPower(Range.clip(-gameyavg - adjustment+gamerighttrig - gamelefttrig,-1,1));
            }
            else
            {
                robo.FLMotor.setPower(speedControl *Range.clip(gamerighttrig - gamelefttrig - gameyright, -1, 1));
                robo.BLMotor.setPower(speedControl *Range.clip(-gamerighttrig + gamelefttrig - gameyright, -1, 1));
                robo.FRMotor.setPower(speedControl *Range.clip(-gamerighttrig + gamelefttrig - gameyleft, -1, 1));
                robo.BRMotor.setPower(speedControl *Range.clip(gamerighttrig - gamelefttrig - gameyleft, -1, 1));
            }

        if (toggleRunmode) {
            if (gamepad1.right_stick_button) {
                fastRunmode = !fastRunmode;
                toggleRunmode = false;
            }
        } else if (!(gamepad1.right_stick_button)) {
            toggleRunmode = true;
        }

        if (toggleBrake) {
            if (gamepad1.a) {
                isBrake = !isBrake;
                toggleBrake = false;
            }
        } else if (!(gamepad1.a)) {
            toggleBrake = true;
        }

        if (isBrake)
        {
            robo.BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robo.BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robo.FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robo.FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else
        {
            robo.BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robo.BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robo.FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robo.FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if (fastRunmode) {
            speedControl = 1;
        } else if (!fastRunmode || gyroMode) {
            speedControl = 0.5;
        }

        robo.rightTrackDown.setPower(0.85*-Range.clip(gamepad2.left_stick_y, -1, 1));
        robo.leftTrackDown.setPower(0.85*-Range.clip(gamepad2.left_stick_y, -1, 1));
        robo.rightTrackUp.setPower(-Range.clip(gamepad2.right_stick_y, -1, 1));
        robo.leftTrackUp.setPower(-Range.clip(gamepad2.right_stick_y, -1, 1));


            if (gamepad2.right_bumper) {
                robo.plate(HardwareTOBOR.platePos.Up);
            } else {
                robo.plate(HardwareTOBOR.platePos.Down);
            }

        if (gamepad2.dpad_up) {
            robo.arm(HardwareTOBOR.armPos.Up);
        } else if (gamepad2.dpad_down) {
            robo.arm(HardwareTOBOR.armPos.Down);
        } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
            robo.arm(HardwareTOBOR.armPos.Back);
        }

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
        robo.plate(HardwareTOBOR.platePos.Down);

    }
}
