package frc.utils;

import edu.wpi.first.wpilibj.RobotController;

/**
*    A basic timer which could be used as a delay 
*    or to set a length of time for a command to
*    execute.
*/
public class CommandTimer {

    private long initTime = 0;
    private long endTime = 0;

    /**
    *    Constructor
    *    @param duration    how long should the timer run for in ms
    */
    public CommandTimer(long duration) {
        initTime = System.currentTimeMillis();
        endTime = initTime + duration;
    }

    /**
    * isComplete
    * Check this function in your isFinished() function or during your execute()
    * function in a command. 
    * @return true if timer is done, else false
    */
    public boolean isComplete() {
        System.out.println("init time: " + initTime + ", end time: " + endTime + ", current time: " + System.currentTimeMillis());
        if(System.currentTimeMillis() >= endTime) {
            return true;
        } 
        return false;
    }
}
