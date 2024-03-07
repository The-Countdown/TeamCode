package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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

    public RobotPosition getPositionNew(VisionPipeline vision1, VisionPipeline vision2, Telemetry telemetry) {
        Map<Integer, String> aprilPosTable = new HashMap<>();

        // Add some key-value pairs to the dictionary
        aprilPosTable.put(10, "36|144");
        aprilPosTable.put(9, "30|144");
        aprilPosTable.put(7, "144|30");
        aprilPosTable.put(8, "144|48");


        List<VisionPipeline.AprilOffset> offsets1 = vision1.getData();
        List<VisionPipeline.AprilOffset> offsets2 = vision2.getData();

        List<VisionPipeline.AprilOffset> allOffsets = new ArrayList<>();
        for (VisionPipeline.AprilOffset offset : offsets1) {
            allOffsets.add(offset);
        }
        for (VisionPipeline.AprilOffset offset : offsets2) {
            allOffsets.add(offset);
        }

        RobotPosition robotpos = new RobotPosition(0, 0);
        double lastx = 0;
        double lasty = 0;

        for (VisionPipeline.AprilOffset offset : allOffsets) {
            telemetry.addData("ftc pose x: ", offset.ftcposx);
            telemetry.addData("ftc pose y: ", offset.ftcposy);
            telemetry.addData("ftc pose z: ", offset.ftcposz);
            int key = offset.id;
            telemetry.addData("April id: ", key);
            String value = aprilPosTable.get(key);
            String[] parts = value.split("\\|");
            double xpos = Integer.parseInt(parts[0]);
            telemetry.addData("x pos int: ", xpos);
            double ypos = Integer.parseInt(parts[1]);
            telemetry.addData("y pos int: ", ypos);

            // Convert the angles to radians
            double pitchRad = Math.toRadians(offset.pitch);
            double rollRad = Math.toRadians(offset.roll);
            double yawRad = Math.toRadians(offset.yaw);

            // Calculate the new x, y, and z coordinates based on the rotation
            double newX = xpos * Math.cos(yawRad) * Math.cos(pitchRad) +
                    ypos * (Math.cos(yawRad) * Math.sin(pitchRad) * Math.sin(rollRad) - Math.sin(yawRad) * Math.cos(rollRad)) +
                    offset.ftcposz * (Math.cos(yawRad) * Math.sin(pitchRad) * Math.cos(rollRad) + Math.sin(yawRad) * Math.sin(rollRad));

            double newY = xpos * Math.sin(yawRad) * Math.cos(pitchRad) +
                    ypos * (Math.sin(yawRad) * Math.sin(pitchRad) * Math.sin(rollRad) + Math.cos(yawRad) * Math.cos(rollRad)) +
                    offset.ftcposz * (Math.sin(yawRad) * Math.sin(pitchRad) * Math.cos(rollRad) - Math.cos(yawRad) * Math.sin(rollRad));

            double newZ = -xpos * Math.sin(pitchRad) +
                    ypos * Math.cos(pitchRad) * Math.sin(rollRad) +
                    offset.ftcposz * Math.cos(pitchRad) * Math.cos(rollRad);

            // Update the position
            newX = xpos + newX;
            newY = xpos + newY;
            newZ = newZ;

            telemetry.addData("stuff: ", String.format("New position: x = " + offset.ftcposx + ", y = " + offset.ftcposy + ", z = " + offset.ftcposz));
            if (lastx == 0) {
                robotpos.x = xpos + newX;
            } else {
//                robotpos.x = (xpos + offset.ftcposx) + lastx / 2;
            }
            telemetry.addData("robot pos x", robotpos.x);
            if (lasty == 0) {
                robotpos.y = ypos + newY;
            } else {
//                robotpos.y = (xpos + offset.ftcposy) + lastx / 2;
            }
            telemetry.addData("robot pos y", robotpos.y);

//            lastx = robotpos.x;
            telemetry.addData("last x", lastx);
//            lasty = robotpos.y;
            telemetry.addData("last y", lasty);
        }

        telemetry.update();
        return robotpos;
    }
        public class RobotPosition {
        public double x;
        public double y;
        public RobotPosition(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
    public class AprilPosition {
        public double x;
        public double y;

        public AprilPosition(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
