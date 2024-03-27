package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Vector;

public class Positioning extends Robot.HardwareDevices {
    public RobotPosition position = new RobotPosition(0, 0, 0);
    private RobotPosition lastPosition = new RobotPosition(0, 0, 0);
    private final VisionPosition visionPosition = new VisionPosition();
    private final EncoderPosition encoderPosition = new EncoderPosition();

    private Robot robot;
    private Telemetry telemetry;
    private VisionPipeline vision1;
    private VisionPipeline vision2;
    private LinearOpMode opMode;

    public Positioning(Robot robot, Telemetry telemetry, VisionPipeline vision1, VisionPipeline vision2, LinearOpMode opMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.vision1 = vision1;
        this.vision2 = vision2;
        this.opMode = opMode;

        startPositioning();
    }

    private void startPositioning() {
        new Thread(() -> {
            while (!opMode.isStopRequested()) {
                try {
                    position = getPosition(telemetry, vision1, vision2);
                } catch (Exception e) {
                    telemetry.addData("Error", e.getMessage());
                    telemetry.update();
                }
            }
        }).start();
    }

    private RobotPosition getPosition(Telemetry telemetry, VisionPipeline vision1, VisionPipeline vision2) {
        RobotPosition vp = visionPosition.getPosition(vision1, vision2, telemetry);
        RobotPosition ep = encoderPosition.getPosition(telemetry);

//        if (vp != null) {
//            lastPosition = vp;
//        } else if (vp == null) {
//            lastPosition = ep;
//        }

        lastPosition = ep;

        if (vp.rot == 0 && ep.rot == 0) {
            lastPosition.rot = 0;
        } else if (ep.rot != 0) {
            lastPosition.rot = ep.rot;
        }
        return lastPosition;
    }

    private class VisionPosition {
        public RobotPosition getPosition(VisionPipeline vision1, VisionPipeline vision2, Telemetry telemetry) {
            Map<Integer, String> aprilPosTable = new HashMap<>();

            int cameraOffset1x = -8;
            int cameraOffset1y = -7;
            int cameraOffset2x = -8;
            int cameraOffset2y = -8;

            // Add some key-value pairs to the dictionary
            aprilPosTable.put(10, "30|0|north");
            aprilPosTable.put(9, "36|0|north");
            aprilPosTable.put(7, "143|113|west");
            aprilPosTable.put(8, "143|108|west");

            List<VisionPipeline.AprilOffset> offsets1 = vision1.getData();
            List<VisionPipeline.AprilOffset> offsets2 = vision2.getData(); // not used right now

            List<VisionPipeline.AprilOffset> allOffsets = new ArrayList<>();
            for (VisionPipeline.AprilOffset offset : offsets1) {
                allOffsets.add(offset);
            }
            for (VisionPipeline.AprilOffset offset : offsets2) {
                allOffsets.add(offset);
            }

            RobotPosition robotpos = new RobotPosition(0, 0, 0);
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
                String direction = parts[2];
                AprilPosition aprilTag = new AprilPosition(xpos, ypos, direction);
                telemetry.addData("aprilTag: ", String.valueOf(aprilTag.x) + " " + String.valueOf(aprilTag.y) + " " + String.valueOf(aprilTag.direction));

                telemetry.addData("Pitch: ", offset.pitch);
                telemetry.addData("Roll: ", offset.roll);
                telemetry.addData("Yaw", offset.yaw);
                if (lastx == 0) {
                    if (Objects.equals(aprilTag.direction, "north")) {
                        robotpos.x = aprilTag.x + Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy + offset.ftcposx;
                    } else if (Objects.equals(aprilTag.direction, "south")) {
                        robotpos.x = - aprilTag.x + Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy + offset.ftcposx;
                    } else if (Objects.equals(aprilTag.direction, "east")) {

                    } else if (Objects.equals(aprilTag.direction, "west")) {
                        if (offset.yaw >= 0) {
                            robotpos.x = aprilTag.x - Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy + offset.ftcposx - offset.ftcposy;
                        } else if (offset.yaw <= 0) {
                            robotpos.x = aprilTag.x - Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy + offset.ftcposx - offset.ftcposy;
                        }
                    }
                } else if (lastx != 0) {
                    if (Objects.equals(aprilTag.direction, "north")) {
                        robotpos.x = aprilTag.x + Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy + offset.ftcposx;
                        robotpos.x = robotpos.x + lastx / 2;
                    } else if (Objects.equals(aprilTag.direction, "south")) {
                        robotpos.x = - aprilTag.x + Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy + offset.ftcposx;
                        robotpos.x = robotpos.x + lastx / 2;
                    } else if (Objects.equals(aprilTag.direction, "east")) {

                    } else if (Objects.equals(aprilTag.direction, "west")) {
                        robotpos.x = aprilTag.x - Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy + offset.ftcposx - offset.ftcposy;
                        robotpos.x = robotpos.x + lastx / 2;
                    }
                }
                telemetry.addData("robot pos x", robotpos.x);
                if (lasty == 0) {
                    if (Objects.equals(aprilTag.direction, "north")) {
                        double inter = Math.tan(Math.toRadians(45)) * offset.ftcposy;
                        robotpos.y = - aprilTag.y + Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy;
                    } else if (Objects.equals(aprilTag.direction, "east")) {

                    } else if (Objects.equals(aprilTag.direction, "west")) {
                        if (offset.yaw >= 0) {
                            robotpos.y = aprilTag.y + Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy;
                        } else if (offset.yaw <= 0) {
                            robotpos.y = aprilTag.y - Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy - offset.ftcposx;
                        }
                    }
                } else if (lasty != 0) {
                    if (Objects.equals(aprilTag.direction, "north")) {
                        double inter = Math.tan(Math.toRadians(45)) * offset.ftcposy;
                        robotpos.y = - aprilTag.y + Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy;
                        robotpos.y = robotpos.y + lasty / 2;
                    } else if (Objects.equals(aprilTag.direction, "east")) {

                    } else if (Objects.equals(aprilTag.direction, "west")) {
                        if (offset.yaw >= 0) {
                            robotpos.y = aprilTag.y + Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy;
                            robotpos.y = robotpos.y + lasty / 2;
                        } else if (offset.yaw <= 0) {
                            robotpos.y = aprilTag.y - Math.tan(Math.toRadians(offset.yaw)) * offset.ftcposy - offset.ftcposx;
                            robotpos.y = robotpos.y + lasty / 2;
                        }
                    }
                }

                // Ajust for the cameras not being at the center of the robot
                robotpos.x = robotpos.x + cameraOffset1x;
                robotpos.y = robotpos.y + cameraOffset1y;

                telemetry.addData("robot pos y", robotpos.y);

                lastx = robotpos.x;
                lasty = robotpos.y;
                telemetry.addData("last x", lastx);
                telemetry.addData("last y", lasty);
            }

            telemetry.update();

            if (robotpos.x == 0 && robotpos.y == 0) {
                return null;
            }
            return robotpos;
        }

        private class AprilPosition {
            public double x;
            public double y;
            public String direction;

            public AprilPosition(double x, double y, String direction) {
                this.x = x;
                this.y = y;
                this.direction = direction;
            }
        }
    }


    private class EncoderPosition {
        private final int wheelDiameter = 90;
        private final double wheelCircumference = Math.PI * wheelDiameter;
        private final double motorRPM = 6000;
        private final double gearRatio = 40;
        private final double ticksPerRev = motorRPM/gearRatio;

        private double lastMotor1Pos = 0;
        private double lastMotor2Pos = 0;
        private double lastMotor3Pos = 0;
        private double lastMotor4Pos = 0;

        public RobotPosition getPosition(Telemetry telemetry) {
            // get position using encoders
            double motor1Pos = MotorFL.getCurrentPosition() - lastMotor1Pos;
            double motor2Pos = MotorFR.getCurrentPosition() - lastMotor2Pos;
            double motor3Pos = MotorBL.getCurrentPosition() - lastMotor3Pos;
            double motor4Pos = MotorBR.getCurrentPosition() - lastMotor4Pos;

            lastMotor1Pos = MotorFL.getCurrentPosition();
            lastMotor2Pos = MotorFR.getCurrentPosition();
            lastMotor3Pos = MotorBL.getCurrentPosition();
            lastMotor4Pos = MotorBR.getCurrentPosition();

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
            RobotPosition position = new RobotPosition(lastPosition.x + avgDistance * Math.cos(currentYawAngle) - sidewaysMovement * Math.sin(currentYawAngle),
                    lastPosition.y + avgDistance * Math.sin(currentYawAngle) + sidewaysMovement * Math.cos(currentYawAngle), currentYawAngle);

            return position;
        }
    }

    public class RobotPosition {
        public double x;
        public double y;
        public double rot;

        public RobotPosition(double x, double y, double rot) {
            this.x = x;
            this.y = y;
            this.rot = rot;
        }
    }
}
