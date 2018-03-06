package org.firstinspires.ftc.team7234.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team7234.common.AutoBase;
import org.firstinspires.ftc.team7234.common.enums.AllianceColor;
import org.firstinspires.ftc.team7234.common.enums.FieldLocation;

@Autonomous(name = "NEWER Blue Far Auto", group = "Inheritance Experiment")
public class BlueFar extends AutoBase {
    public BlueFar(){
        super(AllianceColor.BLUE, FieldLocation.FAR, "BlueFar");
    }
}
