package org.firstinspires.ftc.team7234;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.team7234.common.HardwareBotman;
import org.firstinspires.ftc.team7234.common.RelicVuMarkIdentification2;

@Disabled
@Autonomous(name = "NEW Blue Close", group = "DB")
public class BlueCloseAuto7234 extends OpMode{

    private static final String logTag = BlueCloseAuto7234.class.getName();

    private RelicVuMarkIdentification2 relicVuMark = new RelicVuMarkIdentification2();
    public RelicRecoveryVuMark keyFinder;
    HardwareBotman robot = new HardwareBotman();

    private boolean firstloop;

    public enum currentState {
        PREP,
        JEWEL,
        TWISTCW, TWISTCCW,
        RETURN,
        MOVETOBOX,
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

    private double refLF;
    private double refRF;
    private double refLB;
    private double refRB;

    private String jewelString;

    private double[] deltas;
    private double htarget = 0.0;

    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

    @Override
    public void init() {
        robot.init(hardwareMap, false, DcMotor.ZeroPowerBehavior.BRAKE);
        relicVuMark.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        firstloop = true;
        assignRefererence();
        deltas = robot.mecanumDeltas(0,0);
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

                //This is for the color blue and double checking through the amount of blue so that it doesn't
                //mistake a blue-ish lit room
                if((robot.hsvValues[0] > 175 && robot.hsvValues[0] < 215) && (robot.hsvValues[1] > .5)){
                    state = currentState.TWISTCCW;
                    jewelString = "BLUE";
                    Log.i(logTag, "Jewel Removed, color seen was " + jewelString);
                }
                //This does the same except for the color red
                else if((robot.hsvValues[0] > 250 || robot.hsvValues[0] < 15) && (robot.hsvValues[1] > .5)) {
                    state = currentState.TWISTCW;
                    jewelString = "RED";
                    Log.i(logTag, "Jewel Removed, color seen was " + jewelString);
                }
                break;
            case TWISTCW:
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
            case TWISTCCW:
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
                    deltas = robot.mecanumDeltas(0.0, 37.0);
                    Log.i(logTag, "Return Completed, moving to cryptobox."
                            + "\nCurrent Heading is: "
                            + robot.heading()
                            + "\nReference Motor is now at: "
                            + refLF
                            + "\nTarget is: "
                            + (refLF + -deltas[0])
                    );
                    state = currentState.MOVETOBOX;
                }
                break;
            case MOVETOBOX:
                Log.v(logTag, "Moving to box, current position is: "
                        + -robot.leftFrontDrive.getCurrentPosition()
                        + "\nTarget is: "
                        + (-refLF + -deltas[0])
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

                if (robot.leftFrontDrive.getCurrentPosition() <= refLF + -deltas[0]){
                    robot.mecanumDrive(Math.PI, 0.3, rot);
                }
                else {
                    robot.mecanumDrive(0.0, 0.0, 0.0);
                    assignRefererence();
                    Log.i(logTag, "Box Reached, beginning spin.\nTarget LF was:"
                            + (refLF + -deltas[0])
                            + "\nEnding Value was: "
                            + robot.leftFrontDrive.getCurrentPosition()
                    );
                    state = currentState.ALIGN;
                }
                break;
            case ALIGN:

                if(firstloop){
                    htarget = (robot.heading()-45.0+180.0)%360.0-180.0;
                    firstloop = false;
                }
                else{
                    if (robot.heading() >= htarget - 3.0 && robot.heading() <= htarget +3.0){
                        robot.mecanumDrive(0.0,0.0,0.0);
                        Log.i(logTag, "Alignment Completed, Releasing Glyph.\nCurrent Heading is: "
                                + robot.heading()
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
                robot.gripperSet(gripperState);
                deltas = robot.mecanumDeltas(0.0, -3.0);
                break;
            case RETREAT:
                if(robot.leftBackDrive.getCurrentPosition() >= deltas[0]){
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
        Log.i(logTag, "Autonomous Completed. \nEnding state was: "
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
