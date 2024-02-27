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

            int point2x = 10;
            int point2y = 10;

            // will need to map the angles to points
            double th1 = (90 - (int) angle1.angle);
            double th2 = (90 - (int) angle2.angle);
            th1 = Math.toRadians(th1);
            th2 = Math.toRadians(th2);

            double x = (Math.tan(th1) * point2x) + point2y;
            double y = Math.tan(th2) * x;

//            RobotPosition robotPosition = new RobotPosition(x, y);
//            return (x, y);

        }

        // figure out all the calculation to get field position from here

    }
    public class RobotPosition {
        private double x;
        private double y;
        public void RobotPosition(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
