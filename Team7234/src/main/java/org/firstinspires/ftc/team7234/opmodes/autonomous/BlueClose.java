package org.firstinspires.ftc.team7234.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team7234.common.AutoBase;
import org.firstinspires.ftc.team7234.common.enums.AllianceColor;
import org.firstinspires.ftc.team7234.common.enums.FieldLocation;

@Autonomous(name = "NEWER Blue Close Auto", group = "Inheritance Experiment")
public class BlueClose extends AutoBase {
    public BlueClose(){
        super(AllianceColor.BLUE, FieldLocation.CLOSE, "BlueClose");
    }
}
