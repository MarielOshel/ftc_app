package org.firstinspires.ftc.team7234.common;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.team7234.common.enums.AllianceColor;
import org.firstinspires.ftc.team7234.common.enums.FieldLocation;

import java.util.Arrays;

/**
 * This class is meant to provide a framework on which to build our autonomous for Relic Recovery
 * @author Donald Brown
 */
public class AutoBase extends OpMode{

    private final AllianceColor allianceColor; //Color of the Alliance
    private final FieldLocation fieldLocation; //Location on the field, relative to the recovery zone
    private final String logTag; //Name of program extending this class, for logging purposes

    private final double boxDistY; //Distance Travelled, in inches, to reach the cryptobox in the front-back direction
    private final double boxDistX; //Distance Travelled, in inches, to reach the cryptobox in the left-right direction
    private final double theta1; //Angle of rotation with which to align the robot with the cryptobox
    private final double direction1; //Direction in which to travel when aligning with the cryptobox in the Y axis
    private final double direction2; //Direction in which to travel when aligning with the cryptobox in the X axis


    public AutoBase(AllianceColor allianceColor, FieldLocation fieldLocation, String logTag){
        this.allianceColor = allianceColor;
        this.fieldLocation = fieldLocation;
        this.logTag = logTag;
        if(fieldLocation == FieldLocation.CLOSE && allianceColor == AllianceColor.RED){
            this.boxDistY = -37.0;
            this.boxDistX = 0.0;
            this.theta1 = 45.0;
            this.direction1 = Math.PI;
            this.direction2 = 0.0;
        }
        else if (fieldLocation == FieldLocation.FAR && allianceColor == AllianceColor.RED){
            this.boxDistY = -24.0;
            this.boxDistX = 12.0;
            this.theta1 = 225.0;
            this.direction1 = Math.PI;
            this.direction2 = Math.PI/2.0;
        }
        else if (fieldLocation == FieldLocation.CLOSE && allianceColor ==AllianceColor.BLUE){
            this.boxDistY = 37.0;
            this.boxDistX = 0.0;
            this.theta1 = -45.0;
            this.direction1 = 0.0;
            this.direction2 = 0.0;
        }
        else if (fieldLocation == FieldLocation.FAR && allianceColor == AllianceColor.BLUE){
            this.boxDistY = 24.0;
            this.boxDistX = 12.0;
            this.theta1 = 45.0;
            this.direction1 = 0.0;
            this.direction2 = 3.0*Math.PI/2.0;
        }
        else{
            this.boxDistY = 0.0;
            this.boxDistX = 0.0;
            this.theta1 = 0.0;
            this.direction1 = 0.0;
            this.direction2 = 0.0;
            Log.e("AutoBase", "ERROR: Failed to initialize correctly, ending after Jewel");
        }
    }

    private RelicVuMarkIdentification2 relicVuMark = new RelicVuMarkIdentification2();
    public RelicRecoveryVuMark keyFinder;
    private HardwareBotman robot = new HardwareBotman();

    public enum currentState {
        PREP,
        JEWEL,
        TWIST_CW, TWIST_CCW,
        RETURN,
        MOVE_TO_BOX_Y,
        MOVE_TO_BOX_X,
        ALIGN,
        RELEASE,
        RETREAT

    }

    private currentState state = currentState.PREP;

    private HardwareBotman.GripperState gripperState = HardwareBotman.GripperState.HALFWAY;

    private static final double LEFT_DIST = 0.0; //Distance to move for left box, in inches
    private static final double CENTER_DIST = 0.0; //Distance to move for center box, in inches
    private static final double RIGHT_DIST = 0.0; //Distance to move for right box, in inches

    private boolean keyRead = false;
    private boolean alignStart;

    private double refLF;
    private double refRF;
    private double refLB;
    private double refRB;

    private double[] deltas;
    private double htarget = 0.0;

    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

    @Override
    public void init() {
        robot.init(hardwareMap, false, DcMotor.ZeroPowerBehavior.BRAKE);
        relicVuMark.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

        alignStart = true;
        assignRefererence();
        deltas = robot.mecanumDeltas(0,0);
        Log.i(logTag, "Initalization complete");
    }

    @Override
    public void start(){
        relicVuMark.start();
        Log.i(logTag, "Vumark started");
    }

    @Override
    public void loop(){

        //region TELEMETRY
        telemetry.addData("Program Position: ", state.toString());
        telemetry.addLine();
        telemetry.addData("Gripper Position: ", gripperState.toString());
        telemetry.addData("Key Seen is: ", vuMark.toString());
        //endregion

        if (!keyRead && relicVuMark.readKey() != RelicRecoveryVuMark.UNKNOWN){
            vuMark = relicVuMark.readKey();
            Log.i(logTag, "vuMark Found, state is: " + vuMark.toString());
            keyRead = true;
        }

        switch (state){
            case PREP:
                if (!robot.armLimit.getState()){
                    gripperState = HardwareBotman.GripperState.CLOSED;
                    robot.gripperSet(gripperState);
                    robot.arm.setPower(0.2);
                }
                else {
                    robot.arm.setPower(0.0);
                    robot.jewelPusher.setPosition(HardwareBotman.JEWEL_PUSHER_DOWN);
                    state = currentState.JEWEL;
                    Log.i(logTag, "Preparation Completed, Gripper State is: " + gripperState.toString());
                }
                break;

            case JEWEL:

                //converts RGB Reading of Color Sensor to HSV for better color detection
                Color.RGBToHSV(
                        robot.jewelColorSensor.red()*8,
                        robot.jewelColorSensor.green()*8,
                        robot.jewelColorSensor.blue()*8,
                        robot.hsvValues
                );

                Log.v(logTag, "Searching for Jewel, currently seeing [" + Arrays.toString(robot.hsvValues) + "]");

                //This is for the color blue and double checking through the amount of blue so that it doesn't
                //mistake a blue-ish lit room
                String jewelString;
                if((robot.hsvValues[0] > 175 && robot.hsvValues[0] < 215) && (robot.hsvValues[1] > .5)){
                    switch (allianceColor){
                        case RED:
                            state = currentState.TWIST_CW;
                            jewelString = "BLUE";
                            Log.i(logTag, "Clockwise Turn, color seen was " + jewelString);
                            break;
                        case BLUE:
                            state = currentState.TWIST_CCW;
                            jewelString = "BLUE";
                            Log.i(logTag, "CounterClockwise Turn, color seen was " + jewelString);
                            break;
                    }

                }
                //Else, if it sees a blue jewel
                else if((robot.hsvValues[0] > 250 || robot.hsvValues[0] < 15) && (robot.hsvValues[1] > .5)) {
                    switch (allianceColor){
                        case RED:
                            state = currentState.TWIST_CCW;
                            jewelString = "RED";
                            Log.i(logTag, "CounterClockwise Turn, color seen was " + jewelString);
                            break;
                        case BLUE:
                            state = currentState.TWIST_CW;
                            jewelString = "RED";
                            Log.i(logTag, "Clockwise Turn, color seen was " + jewelString);
                            break;
                    }
                }
                break;
            case TWIST_CW:
                Log.v(logTag, "Now turning, target is -10, current heading is: " + robot.heading());
                if (robot.heading() >= -10){
                    robot.mecanumDrive(0.0, 0.0, 0.3);
                }
                else{
                    robot.jewelPusher.setPosition(HardwareBotman.JEWEL_PUSHER_UP);
                    robot.mecanumDrive(0.0, 0.0, 0.0);
                    Log.i(logTag, "Clockwise Twist Completed, returing to orientation.\nCurrent Heading is: "
                            + robot.heading()
                    );
                    state = currentState.RETURN;
                }
                break;
            case TWIST_CCW:
                Log.v(logTag, "Now turning, target is 10, current heading is: " + robot.heading());
                if (robot.heading() <= 10){
                    robot.mecanumDrive(0.0, 0.0, -0.3);
                }
                else{
                    robot.jewelPusher.setPosition(HardwareBotman.JEWEL_PUSHER_UP);
                    robot.mecanumDrive(0.0, 0.0, 0.0);
                    Log.i(logTag, "Counterclockwise Twist Completed, returing to orientation.\nCurrent Heading is: "
                            + robot.heading()
                    );
                    state = currentState.RETURN;
                }
                break;
            case RETURN:
                if (robot.heading() <= -2.0){
                    robot.mecanumDrive(0.0,0.0, -0.3);
                }
                else if (robot.heading() >= 2.0) {
                    robot.mecanumDrive(0.0, 0.0, 0.3);
                }
                else{
                    robot.mecanumDrive(0.0,0.0,0.0);
                    assignRefererence();
                    deltas = robot.mecanumDeltas(0.0, boxDistY);
                    Log.i(logTag, "Return Completed, moving to cryptobox."
                            + "\nCurrent Heading is: "
                            + robot.heading()
                            + "\nReference Motor is now at: "
                            + refLF
                            + "\nTarget is: "
                            + (refLF + -deltas[0])
                    );
                    state = currentState.MOVE_TO_BOX_Y;
                }
                break;
            case MOVE_TO_BOX_Y:
                Log.v(logTag, "Moving to box in Y axis, current position is: "
                        + robot.leftFrontDrive.getCurrentPosition()
                        + "\nTarget is: "
                        + (refLF + -deltas[0])
                );

                double rot;
                if (robot.heading() < -2.0){
                    rot = -0.2;
                }
                else if (robot.heading() > 2.0){

                    rot = 0.2;
                }
                else{
                    rot = 0.0;
                }

                if (deltas[0] < 0.0){
                    if (robot.leftFrontDrive.getCurrentPosition() <= refLF -deltas[0]){
                        robot.mecanumDrive(direction1, 0.3, rot);
                    }
                    else {
                        robot.mecanumDrive(0.0, 0.0, 0.0);
                        assignRefererence();
                        switch (fieldLocation){
                            case CLOSE:
                                state = currentState.ALIGN;
                                Log.i(logTag, "Box Reached, beginning spin.\nTarget LF was:"
                                        + (refLF + -deltas[0])
                                        + "\nEnding Value was: "
                                        + robot.leftFrontDrive.getCurrentPosition()
                                );
                                break;
                            case FAR:
                                state = currentState.MOVE_TO_BOX_X;
                                Log.i(logTag, "Box Y Reached, X axis.\nTarget LF was:"
                                        + (refLF + -deltas[0])
                                        + "\nEnding Value was: "
                                        + robot.leftFrontDrive.getCurrentPosition()
                                );
                                assignRefererence();

                                deltas = robot.mecanumDeltas(boxDistX, 0.0);
                                break;
                            default:
                                state = currentState.ALIGN;
                                break;
                        }
                    }
                }
                else {
                    if (robot.leftFrontDrive.getCurrentPosition() >= refLF -deltas[0]){
                        robot.mecanumDrive(direction1, 0.3, rot);
                    }
                    else {
                        robot.mecanumDrive(0.0, 0.0, 0.0);
                        assignRefererence();
                        switch (fieldLocation) {
                            case CLOSE:
                                state = currentState.ALIGN;
                                Log.i(logTag, "Box Reached, beginning spin.\nTarget LF was:"
                                        + (refLF + -deltas[0])
                                        + "\nEnding Value was: "
                                        + robot.leftFrontDrive.getCurrentPosition()
                                );
                                break;
                            case FAR:
                                state = currentState.MOVE_TO_BOX_X;
                                Log.i(logTag, "Box Y Reached, X axis.\nTarget LF was:"
                                        + (refLF + -deltas[0])
                                        + "\nEnding Value was: "
                                        + robot.leftFrontDrive.getCurrentPosition()
                                );
                                assignRefererence();

                                deltas = robot.mecanumDeltas(boxDistX, 0.0);
                                break;
                            default:
                                state = currentState.ALIGN;
                                break;
                        }
                    }
                }

                break;
            case MOVE_TO_BOX_X:
                Log.v(logTag, "Moving to box in x axis, current position is: "
                        + robot.leftFrontDrive.getCurrentPosition()
                        + "\nTarget is: "
                        + (refLF + -deltas[0])
                );
                if (robot.heading() < -2.0){
                    rot = -0.2;
                }
                else if (robot.heading() > 2.0){

                    rot = 0.2;
                }
                else{
                    rot = 0.0;
                }
                if (robot.leftFrontDrive.getCurrentPosition() <= refLF + -deltas[0] +20.0
                        && robot.leftFrontDrive.getCurrentPosition() >= refLF - deltas[0] - 20.0){
                    robot.mecanumDrive(direction2, 0.3, rot);
                }
                else {
                    robot.mecanumDrive(0.0, 0.0, 0.0);
                    assignRefererence();
                    Log.i(logTag, "Box Reached, beginning spin.\nTarget LF was:"
                            + (refLF + -deltas[0])
                            + "\nEnding Value was: "
                            + robot.leftFrontDrive.getCurrentPosition()
                    );
                    switch (fieldLocation){
                        case CLOSE:
                            state = currentState.ALIGN;
                            break;
                        case FAR:
                            state = currentState.MOVE_TO_BOX_X;
                            assignRefererence();
                            deltas = robot.mecanumDeltas(boxDistX, 0.0);
                            break;
                        default:
                            state = currentState.ALIGN;
                            break;
                    }
                }
                break;
            case ALIGN:

                if(alignStart){
                    htarget = (robot.heading()+theta1+180.0)%360.0-180.0;
                    alignStart = false;
                }
                else{
                    if (robot.heading() >= htarget - 3.0 && robot.heading() <= htarget +3.0){
                        robot.mecanumDrive(0.0,0.0,0.0);
                        Log.i(logTag, "Alignment Completed, Releasing Glyph.\nCurrent Heading is: "
                                + robot.heading()
                                + "\nTarget Heading was: "
                                + htarget
                        );
                        state = currentState.RELEASE;
                    }
                    else {
                        robot.mecanumDrive(0.0, 0.0,-0.3);
                    }
                }
                break;

            case RELEASE:
                gripperState = HardwareBotman.GripperState.HALFWAY;
                assignRefererence();
                robot.gripperSet(gripperState);
                deltas = robot.mecanumDeltas(0.0, -3.0);
                break;

            case RETREAT:
                if(robot.leftFrontDrive.getCurrentPosition() >= refLF -deltas[0]){
                    robot.mecanumDrive(Math.PI, 0.5, 0.0);
                }
                else{
                    robot.mecanumDrive(0,0,0);
                }
                break;
        } //state switch

    } //loop
    @Override
    public void stop(){
        Log.i(logTag, allianceColor.toString()
                + " "
                + fieldLocation.toString()
                + "Autonomous Completed. \nEnding state was: "
                + state.toString()
                + "\nGripper State was: "
                + gripperState.toString()
                + "\nVumark Found was: "
                + vuMark.toString()
        );
    }

    private void assignRefererence(){
        refLB = robot.leftBackDrive.getCurrentPosition();
        refLF = robot.leftFrontDrive.getCurrentPosition();
        refRB = robot.rightBackDrive.getCurrentPosition();
        refRF = robot.rightFrontDrive.getCurrentPosition();
    }

}
