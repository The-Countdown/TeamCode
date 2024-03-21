package org.firstinspires.ftc.teamcode.subsystems;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;

public class PositionLogger extends Robot.HardwareDevices {
    private Robot robot;
    private String filePath;

    public PositionLogger(Robot robot, String filePath) {
        this.robot = robot;
        this.filePath = filePath;
        startLogging();
    }

    private void logPosition() {
        try {
            FileWriter writer = new FileWriter(filePath, true);
            Positioning.RobotPosition position = robot.robotPosition.getPositionNew(robot.vision1, robot.vision2, robot.telemetry);
            writer.write(position.x + "," + position.y + "\n");
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void startLogging() {
        Timer timer = new Timer();
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                logPosition();
            }
        }, 0, 33);
    }
}