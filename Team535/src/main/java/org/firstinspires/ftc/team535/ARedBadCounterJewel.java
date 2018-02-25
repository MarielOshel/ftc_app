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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name="Red Auto Bad Full", group="Autonomous")
//@Disabled
public class ARedBadCounterJewel extends OpMode
{
    HardwareTOBOR robo = new HardwareTOBOR();
    double heading;
    double red;
    double blue;
    double gray;
    boolean crossedRoad = false;
    HardwareTOBOR.direction dir;
    float hsv[] = {0F,0F,0F};
    float hsv2[] = {0F,0F,0F};
    public enum state{
        READJEWEL,
        HITJEWEL,
        ARMUP,
        DRIVEOFFSTONEPRE,
        DRIVEOFFSTONE,
        CENTERONPOINT,
        SEEKCOLUMN,
        ALIGNBOTFORWARD,
        BACKUPFIRST,
        PLACEBLOCK,
        MOVEFORWARD,
        BACKUP,
        STOPALL
    }

    double TPI = 43;
    double lastPos;
    state currentState = state.READJEWEL;
    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        robo.initRobo(hardwareMap);
        robo.initVuforia();
        robo.startVuforia();
        robo.arm(HardwareTOBOR.armPos.Up);

        
    }


    @Override
    public void init_loop()
    {
        if (robo.readKey() != RelicRecoveryVuMark.UNKNOWN)
        {
            vuMark = robo.readKey();
            telemetry.addData("Vumark Acquired", vuMark);
        }
        robo.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    @Override
    public void start()
    {
        robo.runtime.reset();
        robo.accel  = robo.imu.getLinearAcceleration();
        robo.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robo.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robo.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robo.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robo.BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        
    }

    @Override
    public void loop()
    {
        telemetry.addData("BRMotor", robo.BRMotor.getCurrentPosition());
        switch (currentState){
            case READJEWEL:
                robo.wrist(HardwareTOBOR.wristPos.Open);
                robo.arm(HardwareTOBOR.armPos.Down);
                if (robo.readKey() != RelicRecoveryVuMark.UNKNOWN)
                {
                    vuMark = robo.readKey();
                    telemetry.addData("Vumark Acquired", vuMark);
                }
                if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                {
                    robo.stopVuforia();
                }
                if (robo.runtime.seconds() >= 1) {

                    dir = robo.knockJewel(HardwareTOBOR.color.Red);
                    telemetry.addData("Saturation",robo.hsvValues[1]);
                    telemetry.addData("Hue", robo.hsvValues[0]);
                    telemetry.addData("Direction", robo.knockJewel(HardwareTOBOR.color.Red));
                    if (dir != HardwareTOBOR.direction.Unknown && vuMark != RelicRecoveryVuMark.UNKNOWN) {
                        currentState = state.HITJEWEL;
                        robo.runtime.reset();
                    }
                    else if (robo.runtime.seconds() >= 5 && dir != HardwareTOBOR.direction.Unknown) {
                        robo.runtime.reset();
                        currentState = state.HITJEWEL;
                    }
                    else if (robo.runtime.seconds() >= 5)
                    {
                        currentState = state.ARMUP;
                    }
                }
                break;
            case HITJEWEL:
                robo.stopVuforia();
                robo.angles = robo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (dir == HardwareTOBOR.direction.Left && robo.runtime.seconds() <=0.5)
                {
                    robo.wrist(HardwareTOBOR.wristPos.Closed);
                }

                else if (dir == HardwareTOBOR.direction.Right && robo.runtime.seconds() <=0.5)
                {
                    robo.wrist(HardwareTOBOR.wristPos.Right);
                    
                }
                else 
                {
                    robo.runtime.reset();
                    currentState = state.ARMUP;
                }
                
                
                break;
            case ARMUP:
                robo.stopVuforia();
                robo.wrist(HardwareTOBOR.wristPos.Closed);
                robo.arm(HardwareTOBOR.armPos.Back);
                if (robo.runtime.seconds() >= 0.5) {
                    currentState = state.DRIVEOFFSTONE;
                    robo.runtime.reset();
                }
                break;
            case DRIVEOFFSTONEPRE:
                if (robo.runtime.seconds() <=1.1)
                {
                    heading = robo.strafeRightAuto(0.6,0);
                }
                else
                {
                    heading = robo.strafeRightAuto(0.1,0);
                }
                if (robo.BRMotor.getCurrentPosition() >= (12 * TPI))
            {
                currentState = state.DRIVEOFFSTONE;
            }
                break;
            case DRIVEOFFSTONE:

                    heading = robo.strafeRightAuto(0.1,90);

                Color.RGBToHSV(robo.floorSensorLeft.red() * 255, robo.floorSensorLeft.green() * 255, robo.floorSensorLeft.blue() * 255, hsv2);
                Color.RGBToHSV(robo.floorSensorRight.red() * 255, robo.floorSensorRight.green() * 255, robo.floorSensorRight.blue() * 255, robo.hsvValues);
            if (hsv2[1] >=.45)
            {
                crossedRoad = true;
            }

                if (((30*TPI)-robo.BRMotor.getCurrentPosition()< (0.5*TPI))&&(robo.hsvValues[1]>=0.5))
                {
                    currentState = state.CENTERONPOINT;
                }
                break;
            case CENTERONPOINT:
                robo.followLineBackREd(0.05,90);
                Color.RGBToHSV(robo.floorSensorLeft.red() * 255, robo.floorSensorLeft.green() * 255, robo.floorSensorLeft.blue() * 255, hsv);
                if (hsv[1] >= 0.5)
                {
                    currentState = state.SEEKCOLUMN;
                }
                robo.runtime.reset();
                break;
            case SEEKCOLUMN:
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    if(robo.runtime.seconds() < 3.0 || robo.rangeSensor.getDistance(DistanceUnit.INCH)> 6) {
                        robo.followLineForwardRed(.15, 90, false, robo.floorSensorRight);
                        /*robo.accel = robo.imu.getLinearAcceleration();
                        Log.i("Accel", "Accelx: " + robo.accel.xAccel);*/
                    } else {
                        currentState = state.BACKUPFIRST;
                        lastPos = robo.BRMotor.getCurrentPosition();
                    }
                }
                else if (vuMark == RelicRecoveryVuMark.CENTER|| vuMark == RelicRecoveryVuMark.UNKNOWN) {
                    currentState = state.ALIGNBOTFORWARD;
                    robo.runtime.reset();
                }
                else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    if(robo.runtime.seconds() < 3.0 || robo.rangeSensor.getDistance(DistanceUnit.INCH)> 6) {
                        robo.followLineForwardRed(.15, 90, true, robo.floorSensorLeft);
                    } else {
                        currentState = state.BACKUPFIRST;
                        lastPos = robo.BRMotor.getCurrentPosition();
                    }
                }
                break;
            case ALIGNBOTFORWARD:
                robo.DriveForwardAuto(0.2,90);
                if (robo.runtime.seconds() >=2.5)
                {
                    currentState = state.BACKUPFIRST;
                    lastPos = robo.BRMotor.getCurrentPosition();
                }
                break;
            case BACKUPFIRST:
                robo.DriveBackwardAuto(0.2,90);
                if(robo.BRMotor.getCurrentPosition() -lastPos - (6*TPI)>0)
                {
                    currentState = state.PLACEBLOCK;
                    robo.runtime.reset();
                }
                break;
            case MOVEFORWARD:
                robo.DriveForwardAuto(0.2,90);
                if (robo.runtime.seconds() >= 0.5 && robo.runtime.seconds()<= 1)
                {
                    currentState = state.BACKUP;
                }
                if (robo.runtime.seconds() >= 1.5)
                {
                    currentState = state.BACKUP;
                }
                break;
            case PLACEBLOCK:
                robo.rightTrackUp.setPower(1);
                robo.leftTrackUp.setPower(1);
                robo.BRMotor.setPower(0);
                robo.FRMotor.setPower(0);
                robo.BLMotor.setPower(0);
                robo.FLMotor.setPower(0);
                robo.plate(HardwareTOBOR.platePos.Up);
                if (robo.runtime.seconds()>=1)
                {
                    robo.runtime.reset();
                    currentState = state.MOVEFORWARD;
                    robo.plate(HardwareTOBOR.platePos.Down);
                }

                break;
            case BACKUP:
                robo.DriveBackwardAuto(0.2,90);
                if (robo.runtime.seconds() >= 1 && robo.runtime.seconds() <=1.5)
                {

                    currentState = state.MOVEFORWARD;
                }
                if (robo.runtime.seconds() >=2)
                {
                    currentState = state.STOPALL;
                }
                break;
            case STOPALL:
                robo.plate(HardwareTOBOR.platePos.Down);
                robo.BRMotor.setPower(0);
                robo.FRMotor.setPower(0);
                robo.BLMotor.setPower(0);
                robo.FLMotor.setPower(0);
                robo.rightTrackUp.setPower(0);
                robo.leftTrackUp.setPower(0);

                break;
        }
        telemetry.addData("State", currentState);
        telemetry.addData("1", "heading: " + heading);
        telemetry.addData("Vumark Acquired", vuMark);
        telemetry.addData("2", "time: " + robo.runtime.seconds());
        telemetry.addData("3", "accel: " + robo.accel.xAccel);

    }

    @Override
    public void stop()
    {

    }

}
