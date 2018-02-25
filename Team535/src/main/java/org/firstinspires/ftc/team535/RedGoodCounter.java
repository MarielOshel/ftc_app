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
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@Autonomous(name="Red Auto Good Counter", group="Autonomous")
@Disabled
public class RedGoodCounter extends OpMode
{
    HardwareTOBOR robo = new HardwareTOBOR();
    double heading;
    double red;
    double blue;
    double gray;
    boolean crossedRoad = false;
    HardwareTOBOR.direction dir;
    float hsv[] = {0F,0F,0F};
    public enum state{
        READJEWEL,
        HITJEWELOUT,
        HITJEWELIN,
        ARMUP,
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
    state currentState = state.DRIVEOFFSTONE;
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
        robo.arm(HardwareTOBOR.armPos.Down);
        robo.runtime.reset();
        robo.stopVuforia();
        robo.accel  = robo.imu.getLinearAcceleration();
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

                    dir = robo.knockJewel(HardwareTOBOR.color.Red);
                    telemetry.addData("Direction", robo.knockJewel(HardwareTOBOR.color.Red));
                    if (dir != HardwareTOBOR.direction.Unknown) {
                        currentState = state.DRIVEOFFSTONE;
                    } else if (robo.runtime.seconds() >= 5) {

                        currentState = state.ARMUP;
                    }
                }
                break;
            case HITJEWELOUT:
                robo.angles = robo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (dir == HardwareTOBOR.direction.Left)
                {
                    currentState = state.DRIVEOFFSTONE;
                    robo.runtime.reset();

                }

                else if (dir == HardwareTOBOR.direction.Right)
                {
                    robo.BRMotor.setPower(0.2);
                    robo.FRMotor.setPower(0.2);
                    robo.BLMotor.setPower(0);
                    robo.FLMotor.setPower(0);
                    if (robo.angles.firstAngle <= -5)
                    {
                        currentState = state.ARMUP;
                    }
                }
                else
                {
                    currentState = state.READJEWEL;
                }
                break;
            case HITJEWELIN:
                if (robo.angles.firstAngle < -1)
                {
                    robo.BRMotor.setPower(0);
                    robo.FRMotor.setPower(0);
                    robo.BLMotor.setPower(-0.2);
                    robo.FLMotor.setPower(-0.2);
                    if (robo.angles.firstAngle <=2 &&robo.angles.firstAngle >=-2)
                    {
                        currentState = state.STOPALL;
                    }
                }
                else if (robo.angles.firstAngle >1) {
                    robo.BRMotor.setPower(0);
                    robo.FRMotor.setPower(0);
                    robo.BLMotor.setPower(-0.2);
                    robo.FLMotor.setPower(-0.2);
                    if (robo.angles.firstAngle <= 2 && robo.angles.firstAngle >= -2) {
                        currentState = state.DRIVEOFFSTONE;
                    }
                }
                break;
            case ARMUP:
                robo.arm(HardwareTOBOR.armPos.Back);

                currentState = state.HITJEWELIN;
                break;
            case DRIVEOFFSTONE:
                if (robo.runtime.seconds() >= 0.25)
                {
                    robo.arm(HardwareTOBOR.armPos.Back);
                }
                heading = robo.strafeRightAuto(0.3,0);
                Color.RGBToHSV(robo.floorSensorRight.red() * 255, robo.floorSensorRight.green() * 255, robo.floorSensorRight.blue() * 255, robo.hsvValues);
                if (((20*TPI)+robo.BRMotor.getCurrentPosition()< (0.5*TPI))&&(robo.hsvValues[1]>=0.45))
                {
                    currentState = state.CENTERONPOINT;
                }
                break;
            case CENTERONPOINT:
                robo.followLineBackREd(0.25,0);
                Color.RGBToHSV(robo.floorSensorLeft.red() * 255, robo.floorSensorLeft.green() * 255, robo.floorSensorLeft.blue() * 255, hsv);
                if (hsv[1] >= 0.6)
                {
                    crossedRoad = true;
                    currentState = state.SEEKCOLUMN;
                }
                robo.runtime.reset();
                break;
            case SEEKCOLUMN:
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    if(robo.runtime.seconds() < 2.0 || robo.rangeSensor.getDistance(DistanceUnit.INCH)> 6) {
                        robo.followLineForwardRed(.25, 0, false, robo.floorSensorRight);
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
                    if(robo.runtime.seconds() < 2.0 || robo.rangeSensor.getDistance(DistanceUnit.INCH)> 6) {
                        robo.followLineForwardRed(.25, 0, true, robo.floorSensorLeft);
                    } else {
                        currentState = state.BACKUPFIRST;
                        lastPos = robo.BRMotor.getCurrentPosition();
                    }
                }
                break;
            case ALIGNBOTFORWARD:
                robo.DriveForwardAuto(0.2,0);
                if (robo.runtime.seconds() >=1.5)
                {
                    currentState = state.BACKUPFIRST;
                    lastPos = robo.BRMotor.getCurrentPosition();
                }
                break;
            case BACKUPFIRST:
                robo.DriveBackwardAuto(0.2,0);
                if(robo.BRMotor.getCurrentPosition() -lastPos + (4*TPI)<0)
                {
                    currentState = state.PLACEBLOCK;
                    robo.runtime.reset();


                }
                break;
            case MOVEFORWARD:
                robo.DriveForwardAuto(0.2,0);
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
                if (robo.runtime.seconds()>=1)
                {
                    robo.runtime.reset();
                    currentState = state.MOVEFORWARD;
                }

                break;
            case BACKUP:
                robo.DriveBackwardAuto(0.2,0);
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
