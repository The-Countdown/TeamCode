package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Positioning {
    public RobotPosition getPosition(VisionPipeline vision1, VisionPipeline vision2, Telemetry telemetry) {
        List<VisionPipeline.CameraAngle> angle1List = vision1.aprilTagPos();
        List<VisionPipeline.CameraAngle> angle2List = vision2.aprilTagPos();
        if (angle1List.size() < 1 || angle2List.size() < 1) {
            return null;
            // error handing here
        } else {
            VisionPipeline.CameraAngle angle1 = angle1List.get(0);
            telemetry.addData("Angle 1: ", angle1.angle);
            VisionPipeline.CameraAngle angle2 = angle2List.get(0);
            telemetry.addData("Angle 2: ", angle2.angle);

            int point1x = 0;
            int point1y = 0;
            int point2x = 10;
            int point2y = 10;

            // will need to map the angles to points
            double th1 = (90 - (int) angle1.angle);
            double th2 = (90 - (int) angle2.angle);
            th1 = Math.toRadians(th1);
            th2 = Math.toRadians(th2);

//            (y - p2x) * tan(th2) + p2y = (y - p1x) * tan(th1) + p1y
//            (y - p2x) * k + p2y = (y - p1x) * p + p1y


            double p = Math.tan(th1);
            double k = Math.tan(th2);
            double g = point2x;
            double i = point2y;
            double z = point1x;
            double v = point1y;

            //double y = ((point2y + (point1x - point2x) * Math.tan(th2)) - point1y) / Math.tan(th1) + point1x;
            double y = (v + g * Math.tan(p) - z * Math.tan(k))/(Math.tan(p) - Math.tan(k));

            double x = Math.tan(th1) * y;

            RobotPosition robotPosition = new RobotPosition(x, y);
            telemetry.addData("RobotPosition X: ", robotPosition.x);
            telemetry.addData("RobotPosition Y: ", robotPosition.y);
            telemetry.update();
            return (robotPosition);


        }

        // figure out all the calculation to get field position from here

    }
    public class RobotPosition {
        private double x;
        private double y;
        public RobotPosition(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
