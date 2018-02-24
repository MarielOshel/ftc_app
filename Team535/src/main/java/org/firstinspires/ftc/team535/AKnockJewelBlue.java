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

package org.firstinspires.ftc.team535;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name="Knock Blue Jewel", group="Autonomous")
//@Disabled
public class AKnockJewelBlue extends OpMode
{
    HardwareTOBOR robo = new HardwareTOBOR();
    double heading;
    HardwareTOBOR.direction dir = HardwareTOBOR.direction.Unknown;
    public enum state{
        READJEWEL,
        HITJEWELOUT,
        HITJEWELIN,
        ARMUP,
        STOPALL
    }

    double TPI = 43;
    state currentState = state.READJEWEL;
    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

    @Override
    public void init()
    {

        telemetry.addData("Status", "Initialized");
        robo.initRobo(hardwareMap);
        robo.arm(HardwareTOBOR.armPos.Up);
        robo.wrist(HardwareTOBOR.wristPos.Closed);

        
    }


    @Override
    public void init_loop()
    {
        robo.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    @Override
    public void start()
    {
        robo.runtime.reset();
        robo.BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robo.BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robo.FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robo.FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
    }

    @Override
    public void loop()
    {
        telemetry.addData("BRMotor", robo.BRMotor.getCurrentPosition());
        switch (currentState){
            case READJEWEL:
                robo.arm(HardwareTOBOR.armPos.Down);
                if (robo.runtime.seconds() >= 1) {

                    dir = robo.knockJewel(HardwareTOBOR.color.Blue);
                    telemetry.addData("Direction", robo.knockJewel(HardwareTOBOR.color.Blue));
                    if (dir != HardwareTOBOR.direction.Unknown) {
                        currentState = state.HITJEWELOUT;
                    } else if (robo.runtime.seconds() >= 28) {

                        currentState = state.STOPALL;
                    }
                }
                break;
            case HITJEWELOUT:
                robo.angles = robo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (dir == HardwareTOBOR.direction.Left)
                {
                    robo.wrist(HardwareTOBOR.wristPos.Closed);
                    currentState = state.ARMUP;
                }
                else if (dir == HardwareTOBOR.direction.Right)
                {
                    robo.wrist(HardwareTOBOR.wristPos.Right);
                    currentState = state.ARMUP;
                }
                break;
            case ARMUP:
                robo.arm(HardwareTOBOR.armPos.Up);

                currentState = state.STOPALL;
                break;
            case STOPALL:
                robo.plate(HardwareTOBOR.platePos.Down);
                robo.arm(HardwareTOBOR.armPos.Up);
                robo.wrist(HardwareTOBOR.wristPos.Closed);
                robo.BRMotor.setPower(0);
                robo.FRMotor.setPower(0);
                robo.BLMotor.setPower(0);
                robo.FLMotor.setPower(0);
            
            break;
        }
    }

    @Override
    public void stop()
    {

    }

}
