package org.firstinspires.ftc.teamcode.util;

import java.util.TimerTask;

public class TaskCallbackTimer extends TimerTask {
    VoidCallback callbackFunction = null;
    public TaskCallbackTimer(VoidCallback function) {
        callbackFunction = function;
    }
    @Override
    public void run() {
        callbackFunction.call();
    }
}
