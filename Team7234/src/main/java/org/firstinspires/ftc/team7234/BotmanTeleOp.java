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

package org.firstinspires.ftc.team7234;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="BotmanTeleOp", group="Pushbot")
//@Disabled
public class BotmanTeleOp extends OpMode{

    /* Declare OpMode members. */
    HardwareBotman robot       = new HardwareBotman();
    //region Local Variable Declaration
    //Declares the power scaling of the robot
    private static final double driveCurve = 1.0;
    private double driveMultiplier = 1.0;
    private boolean isMecanum;

    private boolean mecanumToggle;
    private boolean gripperClosed;
    private boolean gripperToggle;
    private boolean speedControl;
    private boolean speedToggle;
    //endregion

    @Override
    public void init() {
        robot.init(hardwareMap);
        //region Boolean Initialization

        //Controlling Booleans
        isMecanum = true;
        speedControl = false;
        gripperClosed = true;

        //Toggle Booleans
        mecanumToggle = true;
        gripperToggle = true;
        speedToggle = true;

        //endregion

    }
    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop() {
        //region Drive Variables

        //calculates angle in radians based on joystick position, reports in range [-Pi/2, 3Pi/2]
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (Math.PI / 2);
        if (Double.isNaN(angle)){
            angle = 0;              //Prevents NaN error later in the Program
        }
        //calculates robot speed from the joystick's distance from the center
        double magnitude = driveMultiplier*Math.pow(Range.clip(Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)), 0, 1), driveCurve);
        // How much the robot should turn while moving in that direction
        double rotation = driveMultiplier*Range.clip(gamepad1.right_stick_x, -1, 1);

        //Variables for tank drive
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        //Variable for arm control
        double armPower = gamepad2.left_trigger - gamepad2.right_trigger;

        //region Toggles
        if (mecanumToggle){ //Toggles drive mode based on the x button
            if (gamepad1.x){
                isMecanum = !isMecanum;
                mecanumToggle = false;
            }
        }
        else if (!gamepad1.x) {
            mecanumToggle = true;
        }

        //toggles gripper open/closed based on the gamepad 2 a button
        if (gripperToggle){
            if (gamepad2.a){
                gripperClosed = !gripperClosed;
                gripperToggle = false;
            }
        }
        else if (!gamepad2.a){
            gripperToggle = true;
        }

        if (speedToggle){
            if(gamepad1.b){
                speedControl = !speedControl;
                speedToggle = false;
            }
        }
        else if (!gamepad1.b){
            speedToggle = true;
        }
        //endregion
        driveMultiplier = speedControl ? 0.5 : 1;

        //endregion

        //region Robot Control
        //Sends Power to the Robot Arm
        robot.arm.setPower(armPower);
        //Drives the robot

        if (isMecanum){
            robot.mecanumDrive(angle, magnitude, rotation); //Drives Omnidirectionally
        }
        else{
            if(!gamepad1.right_bumper && !gamepad1.left_bumper){ //Drives as tank
                robot.arrayDrive(left, left, right, right);
            }
            else if (gamepad1.right_bumper && !gamepad1.left_bumper) {  //Strafe right
                robot.arrayDrive(-1, 1, -1, 1);
            }
            else if (!gamepad1.right_bumper) { //Strafe Left
                robot.arrayDrive(1, -1, 1, -1);
            }
            else{
                robot.arrayDrive(0, 0, 0, 0); //Stop
            }
        }

        //endregion
        //region Gripper Control


        if (!gripperClosed){
            robot.gripperOpen();
        }
        else{
            robot.gripperClose();
        }

        //endregion
        //region Telemetry

        telemetry.addData("isMecanum: ", isMecanum);
        telemetry.addData("gripperClosed: ", gripperClosed);
        telemetry.addData("Speed Limited to: ", driveMultiplier);
        telemetry.addLine();
        telemetry.addData("Angle: ", angle);
        telemetry.addData("Magnitude: ", magnitude);
        telemetry.addData("Rotation: ", rotation);
        telemetry.addLine();
        telemetry.addData("X: ", gamepad1.left_stick_x);
        telemetry.addData("Y: ", gamepad1.left_stick_y);
        telemetry.addLine();
        telemetry.addData("FL: ", robot.mecanumSpeeds[0]);
        telemetry.addData("FR: ", robot.mecanumSpeeds[1]);
        telemetry.addData("BR: ", robot.mecanumSpeeds[2]);
        telemetry.addData("BL: ", robot.mecanumSpeeds[3]);
        telemetry.addLine();
        telemetry.addData("FL-pow: ", robot.driveMotors[0].getPower());
        telemetry.addData("FR-pow: ", robot.driveMotors[1].getPower());
        telemetry.addData("BR-pow: ", robot.driveMotors[2].getPower());
        telemetry.addData("BL-pow: ", robot.driveMotors[3].getPower());

        //endregion
    }
    @Override
    public void stop(){}
}
