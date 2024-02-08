package org.firstinspires.ftc.teamcode.subsystems;

public class LinearSlide extends Robot.Components {
    public void stop() {
        LinearSlideR.setVelocity(0);
        LinearSlideL.setVelocity(0);
    }
    public void move(double velocity) {
        LinearSlideR.setVelocity(velocity);
        LinearSlideL.setVelocity(velocity);
    }

    public void move(double velocity, double amount) {
        move(velocity);

        while (LinearSlideR.getCurrentPosition() < amount) {
            //wait
        }

        stop();
    }
}
