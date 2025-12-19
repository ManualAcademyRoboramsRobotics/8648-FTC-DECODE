package org.firstinspires.ftc.teamcode.util.AutoStateManager;


import org.firstinspires.ftc.teamcode.util.VoidCallback;
@FunctionalInterface
interface StateCallback{
    void onComplete();
}
public class AutoState {

    public AutoState(AutoState nextState){

    }

    public void RunTask(VoidCallback NextStateCallback){


        NextStateCallback.call();
    }
}
