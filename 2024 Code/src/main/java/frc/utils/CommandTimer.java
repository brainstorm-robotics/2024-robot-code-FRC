package frc.utils;

import edu.wpi.first.wpilibj.RobotController;

public class CommandTimer {

    private long initTime = 0;
    private long endTime = 0;

    public CommandTimer(long duration) {
        initTime = System.currentTimeMillis();
        endTime = initTime + duration;
    }

    public boolean isComplete() {
        System.out.println("init time: " + initTime + ", end time: " + endTime + ", current time: " + System.currentTimeMillis());
        if(System.currentTimeMillis() >= endTime) {
            return true;
        } 
        return false;
    }
}
