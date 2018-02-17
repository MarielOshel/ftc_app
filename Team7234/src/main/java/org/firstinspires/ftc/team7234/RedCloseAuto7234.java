package org.firstinspires.ftc.team7234;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name = "Red Close", group = "DB")
public class RedCloseAuto7234 extends OpMode{

    private RelicVuMarkIdentification2 relicVuMark = new RelicVuMarkIdentification2();
    public RelicRecoveryVuMark keyFinder;
    HardwareBotman robot = new HardwareBotman();

    public enum currentState {
        PREP,
        JEWEL,
        TWISTCW, TWISTCCW,
        RETURN,
        MOVETOBOX,
        ALIGN,

    }
    private currentState state = currentState.PREP;

    private static final double LEFT_DIST = 0.0; //Distance to move for left box, in inches
    private static final double CENTER_DIST = 0.0; //Distance to move for center box, in inches
    private static final double RIGHT_DIST = 0.0; //Distance to move for right box, in inches

    private boolean keyRead = false;

    private double refLF;
    private double refRF;
    private double refLB;
    private double refRB;

    private double targetAVG;


    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

    @Override
    public void init() {
        robot.init(hardwareMap, false, DcMotor.ZeroPowerBehavior.BRAKE);
        relicVuMark.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start(){
        relicVuMark.start();
    }
    @Override
    public void loop(){

        if (!keyRead && relicVuMark.readKey() != RelicRecoveryVuMark.UNKNOWN){
            vuMark = relicVuMark.readKey();
            keyRead = true;
        }

        switch (state){
            case PREP:
                if (!robot.armLimit.getState()){
                    robot.gripperSet(HardwareBotman.GripperState.CLOSED);
                    robot.arm.setPower(0.2);
                }
                else {
                    robot.arm.setPower(0.0);
                    robot.jewelPusher.setPosition(HardwareBotman.JEWEL_PUSHER_DOWN);
                    state = currentState.JEWEL;
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
                    state = currentState.TWISTCW;
                }
                //This does the same except for the color red
                else if((robot.hsvValues[0] > 250 || robot.hsvValues[0] < 15) && (robot.hsvValues[1] > .5)) {
                    state = currentState.TWISTCCW;
                }
                break;
            case TWISTCW:
                if (robot.heading() >= -10){
                    robot.mecanumDrive(0.0, 0.0, 0.3);
                }
                else{
                    robot.jewelPusher.setPosition(HardwareBotman.JEWEL_PUSHER_UP);
                    robot.mecanumDrive(0.0, 0.0, 0.0);
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

                    refLB = robot.leftBackDrive.getCurrentPosition();
                    refLF = robot.leftFrontDrive.getCurrentPosition();
                    refRB = robot.rightBackDrive.getCurrentPosition();
                    refRF = robot.rightFrontDrive.getCurrentPosition();



                    state = currentState.MOVETOBOX;
                }
                break;
            case MOVETOBOX:
                if (robot.heading() < -2.0){
                    double angle = -0.2;
                }
                else if (robot.heading() > 2.0){
                    double angle = 0.2;
                }
                else{
                    double angle = 0.0;
                }

        }

    }

}
