package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Positioning extends Robot.HardwareDevices {
    private final int wheelDiameter = 90;
    private final double wheelCircumference = Math.PI * wheelDiameter;
    private final double motorRPM = 6000;
    private final double gearRatio = 40;
    private final double ticksPerRev = motorRPM/gearRatio;
    private RobotPosition startingPosition = new RobotPosition(0, 0);
    public RobotPosition getPositionEncoder(Telemetry telemetry) {
        // get position using encoders
        double motor1Pos = MotorFL.getCurrentPosition();
        double motor2Pos = MotorFR.getCurrentPosition();
        double motor3Pos = MotorBL.getCurrentPosition();
        double motor4Pos = MotorBR.getCurrentPosition();

        double motor1Vel = MotorFL.getVelocity();
        double motor2Vel = MotorFR.getVelocity();
        double motor3Vel = MotorBL.getVelocity();
        double motor4Vel = MotorBR.getVelocity();

        double distance1 = motor1Pos / ticksPerRev * wheelCircumference;
        double distance2 = motor2Pos / ticksPerRev * wheelCircumference;
        double distance3 = motor3Pos / ticksPerRev * wheelCircumference;
        double distance4 = motor4Pos / ticksPerRev * wheelCircumference;

        // Calculate the average distance
        double avgDistance = (distance1 + distance2 + distance3 + distance4) / 4;

        // Calculate the average velocity
        double avgVelocity = (motor1Vel + motor2Vel + motor3Vel + motor4Vel) / 4;

        // Get the current yaw angle from the IMU
        double currentYawAngle = imu.getAngularOrientation().firstAngle;

        // Calculate the sideways movement based on the difference in velocities of the front and back motors
        double sidewaysMovement = (motor1Vel - motor3Vel + motor2Vel - motor4Vel) / 4;

        // Update the robot's position based on the average distance, current yaw angle, and sideways movement
        RobotPosition position = new RobotPosition(startingPosition.x + avgDistance * Math.cos(currentYawAngle) - sidewaysMovement * Math.sin(currentYawAngle),
                startingPosition.y + avgDistance * Math.sin(currentYawAngle) + sidewaysMovement * Math.cos(currentYawAngle));

        return position;
    }
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
        aprilPosTable.put(10, "33|0");
        aprilPosTable.put(9, "39|0");
        aprilPosTable.put(7, "143|103");
        aprilPosTable.put(8, "143|100");


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

//            robposx = xpos + ftcposex * (cos(yaw) * cos(pitch) + sin(roll) * sin(yaw) * sin(pitch)) - ftcposey * (cos(pitch) * sin(yaw) - cos(yaw) * sin(roll) * sin(pitch)) + ftcposez * cos(roll) * sin(pitch)
//            robposy = ypos + ftcposex * (cos(roll) * sin(yaw)) + ftcposey * (cos(yaw) * cos(roll)) - ftcposez * sin(roll)

            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", offset.pitch, offset.roll, offset.yaw));
            if (lastx == 0) {
                robotpos.x = xpos + Math.tan(offset.yaw) * offset.ftcposy + offset.ftcposx;
                //                robotpos.x = robotPosX;

            } else {
//                robotpos.x = (xpos + offset.ftcposx) + lastx / 2;
            }
            telemetry.addData("robot pos x", robotpos.x);
            if (lasty == 0) {
                robotpos.y = ypos + Math.tan(offset.yaw) * offset.ftcposy;
//                robotpos.y = robotPosY;
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
