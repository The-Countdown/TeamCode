package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;

public class PositionLogger extends Robot.HardwareDevices {
    private Robot robot;
    private String filePath;
    private LinearOpMode opMode;
    private Timer timer;

    public PositionLogger(Robot robot, String filePath, LinearOpMode opMode) {
        this.robot = robot;
        this.filePath = filePath;
        this.opMode = robot.opMode;
        startLogging();
    }

    private void logPosition() {
        try {
            FileWriter writer = new FileWriter(filePath, true);
            Positioning.RobotPosition position = robot.robotPosition.position;
            writer.write(position.x + "," + position.y + "," + position.rot + "\n");
            writer.close();
        } catch (IOException e) {
            robot.telemetry.addData("Error", e.getMessage());
            robot.telemetry.update();
        }
    }

    private void startLogging() {
        timer = new Timer();
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                logPosition();
            }
        }, 0, 33);
    }
}