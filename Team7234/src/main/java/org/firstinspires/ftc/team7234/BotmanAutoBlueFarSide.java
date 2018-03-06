/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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

package org.firstinspires.ftc.team7234;
//This imports all of the necessary modules and the like that are needed for this program
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.team7234.common.HardwareBotman;
import org.firstinspires.ftc.team7234.common.RelicVuMarkIdentification2;

import static com.sun.tools.javac.util.Constants.format;


@Autonomous(name = "Botman Auto Blue Far", group = "Example")
//@Disabled
public class BotmanAutoBlueFarSide extends OpMode {

    //Sets up classes and variables for later use
    RelicVuMarkIdentification2 relicVuMark = new RelicVuMarkIdentification2();
    public RelicRecoveryVuMark keyFinder;
    HardwareBotman robot = new HardwareBotman();
    double target;


    //This sets up an enumeration statement that we use to run the robot
    //It can be set up as a vertical or horizontal line, and it usually is, but we
    //decided to do it like this so that we can keep track of decisions done by the robot
    currentState programState = currentState.KEY;
    public enum currentState {
        KEY,
        JEWELS,
        TWIST_FORWARD, TWIST_BACKWARD,
        MOVE,
        MOVE_RIGHT,
        LEFT, CENTER, RIGHT,
        SCORE,
        BACKUP
    }

    //This initializes our robot through our hardware map as well as
    //sets up our camera as a sensor
    @Override
    public void init() {
        robot.init(hardwareMap, true, DcMotor.ZeroPowerBehavior.FLOAT);
        relicVuMark.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() { }


    @Override
    public void start() {
        relicVuMark.start();

    }


    //The loop is where one could say "The actual autonomous begins"
    @Override
    public void loop() {
        //We start by declaring a variable called keyFinder in which we use to
        //find out where we need to put the glyph for the extra points
        keyFinder = relicVuMark.readKey();
        if (relicVuMark.vuMark != RelicRecoveryVuMark.UNKNOWN) {

            telemetry.addData("VuMark", "%s visible", keyFinder);
        } else {
            telemetry.addData("VuMark", "not visible");
        }
        relicVuMark.vuMark = RelicRecoveryVuMark.from(relicVuMark.relicTemplate);
        telemetry.addData("Encoder:", robot.leftBackDrive.getCurrentPosition());
        telemetry.addData("Case:", programState);
        telemetry.addData("Hue:", robot.hsvValues[0]);
        telemetry.addData("Saturation:", robot.hsvValues[1]);
        telemetry.addData("Value:", robot.hsvValues[2]);
        switch (programState) {

            //All this case does is show us some telemetry of what the camera picks up
            case KEY:
                if(!robot.armLimit.getState()){
                    telemetry.addData("We are seeing", keyFinder);
                    robot.leftClaw.setPosition(robot.LEFT_GRIPPER_CLOSED);
                    robot.rightClaw.setPosition(robot.RIGHT_GRIPPER_CLOSED);
                    robot.arm.setPower(0.2);
                }

                else{
                    robot.arm.setPower(0);
                    programState = currentState.JEWELS;
                }
                break;

            //This case detects the color of the jewel and switches cases accordingly
            case JEWELS:
                //This line converts RGB to HSV which allows for more accurate detection of color
                Color.RGBToHSV(robot.jewelColorSensor.red() * 8, robot.jewelColorSensor.green() * 8, robot.jewelColorSensor.blue() * 8, robot.hsvValues);

                robot.jewelPusher.setPosition(robot.JEWEL_PUSHER_DOWN);

                //This is for the color blue and double checking through the amount of blue so that it doesn't
                //mistake a blue-ish lit room
                if((robot.hsvValues[0] > 175 && robot.hsvValues[0] < 215) && (robot.hsvValues[1] > .5)){
                    programState = currentState.TWIST_BACKWARD;
                }
                //This does the same except for the color red
                else if((robot.hsvValues[0] > 250 || robot.hsvValues[0] < 15) && (robot.hsvValues[1] > .5)) {
                    programState = currentState.TWIST_FORWARD;
                }
                break;

            //This case twists the robot forward and then returns it to its original position
            case TWIST_FORWARD:
                if(robot.heading() >= -15){
                    robot.arrayDrive(0.3, -0.3, 0.3, -0.3);
                }
                else{
                    robot.arrayDrive(0,0,0,0);
                    robot.jewelPusher.setPosition(robot.JEWEL_PUSHER_UP);
                    target = robot.leftBackDrive.getCurrentPosition();
                    programState = currentState.MOVE;
                }
                break;

            //This case twists the robot backward and then returns it to its original position
            case TWIST_BACKWARD:
                if(robot.heading() <= 10){
                    robot.arrayDrive(-0.3, 0.3, -0.3, 0.3);
                }
                else if (robot.heading() >= -15){
                    robot.jewelPusher.setPosition(robot.JEWEL_PUSHER_UP);
                    robot.arrayDrive(0,0,0,0);
                    target = robot.leftBackDrive.getCurrentPosition();
                    programState = currentState.MOVE;
                }
                break;

            //This case simply moves the robot forward 8 inches
            case MOVE:
                if (robot.leftBackDrive.getCurrentPosition() >= target - 500){
                    robot.driveByGyro(0.3, -15);
                }
                else{
                    robot.arrayDrive(0,0,0,0);
                    target = robot.leftBackDrive.getCurrentPosition();
                    programState = currentState.MOVE_RIGHT;
                }
                break; //remove after testing
                /*
                robot.arrayDrive(1, 1, 1, 1);

                if (robot.leftBackDrive.getCurrentPosition() >= Math.abs(robot.ticsPerInch(12))){
                    robot.mecanumDrive(0, 0, 0);
                }
                else{
                    robot.resetEncoders();
                    if (keyFinder.equals("L")){
                        programState = currentState.LEFT;
                    }
                    else if (keyFinder.equals("C")){
                        programState = currentState.CENTER;
                    }
                    else if (keyFinder.equals("R")){
                        programState = currentState.RIGHT;
                    }
                }
                break;*/
            case MOVE_RIGHT:
                robot.arrayDrive(0,0,0,0);
                programState = currentState.SCORE;
                break;

            /*case LEFT:


            case CENTER:


            case RIGHT:*/


            case SCORE:
                robot.leftClaw.setPosition(robot.LEFT_GRIPPER_OPEN);
                robot.rightClaw.setPosition(robot.RIGHT_GRIPPER_OPEN);

                if (robot.leftBackDrive.getCurrentPosition() <= target + robot.ticsPerInch(3)){
                    robot.arrayDrive(0.5,0.5,0.5,0.5);
                }
                else{
                    robot.arrayDrive(0,0,0,0);
                    target = robot.leftBackDrive.getCurrentPosition();
                    programState = currentState.BACKUP;
                }

                break;

            case BACKUP:
                if (robot.leftBackDrive.getCurrentPosition() >= target + robot.ticsPerInch(-2)){
                    robot.arrayDrive(0.5,0.5,0.5,0.5);
                }
                else{
                    robot.arrayDrive(0,0,0,0);
                }
                break;
        }


    }


    @Override
    public void stop() { }
}
