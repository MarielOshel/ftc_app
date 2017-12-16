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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Botman Auto Red Close", group = "Example")
//@Disabled
public class BotmanAutoRedClose extends OpMode {

    RelicVuMarkIdentification2 relicVuMark = new RelicVuMarkIdentification2();
    public RelicRecoveryVuMark keyfinder;
    HardwareBotman robot = new HardwareBotman();

    //Allows up to remember which key we read

    currentState programState = currentState.KEY;
    public enum currentState {
        KEY,
        JEWELS,
        MOVE,
        TURN_AND_ADJUST,
        SCORE
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        relicVuMark.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() { }


    @Override
    public void start() {
        relicVuMark.start();

    }


    @Override
    public void loop() {
        keyfinder = relicVuMark.readKey();
        relicVuMark.vuMark = RelicRecoveryVuMark.from(relicVuMark.relicTemplate);
        switch (programState) {

            case KEY:

                telemetry.addData("We are seeing ", keyfinder);
                break;

            case JEWELS:
                robot.jewelPusher.setPosition(.6);
                Color.RGBToHSV(robot.jewelColorSensor.red() * 8, robot.jewelColorSensor.green() * 8, robot.jewelColorSensor.blue() * 8, robot.hsvValues);
                telemetry.addData("HSV is", robot.hsvValues );

                if (robot.hsvValues = ); //TODO: Ricky Fix this

                else

                    break;
                break;

            case MOVE:
                //Manuever infront of the box
                while (robot.leftBackDrive.getCurrentPosition() < Math.abs(5000))
                break;

            case TURN_AND_ADJUST:
               // if
                    //Line up for left
               // }
                //if
                    //Line up for center
               // }
               // if
                    //Line up for right
               // }
                break;

            case SCORE:
                //Score glyph
        }


    }


    @Override
    public void stop() { }
}
