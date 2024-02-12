package org.firstinspires.ftc.teamcode.subsystems;

import java.util.List;

public class Positioning {
    public void getPosition(VisionPipeline vision1, VisionPipeline vision2) {
        List<VisionPipeline.CameraAngle> angle1List = vision1.aprilTagPos();
        List<VisionPipeline.CameraAngle> angle2List = vision1.aprilTagPos();
        if (angle1List.size() > 1 || angle2List.size() > 1) {
            return;
            // error handing here
        } else {
            VisionPipeline.CameraAngle angle1 = angle1List.get(0);
            VisionPipeline.CameraAngle angle2 = angle2List.get(0);
        }

        // figure out all the calculation to get field position from here

    }
}
