package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class controls {
    //Logitech large joystick
    private static final Joystick logA = new Joystick(0);
    //Xbox controller
    private static final Joystick xbox = new Joystick(1);
    //Logitech controller
    private static final Joystick logi = new Joystick(2);
    public static void setup(){
        logA.setZChannel(4);
        logA.setThrottleChannel(3);
        logA.setTwistChannel(2);
    }
    //Xbox indexes based on following chart
    //https://lh3.googleusercontent.com/proxy/evdqSkui7r-NKGg678aqL05sNj5gMnzkqam7rQmciUfh8sOb1SfrTTj5B454JXrq3lqrNm7kXVO7f03I_XpCbpRWYESWMEmfrXdibQhkIdqW43bhsDrd214bhbqXQ85-aQyB__lg0eEsiWvDzuodr2zApp92
    public static double[] getXboxAnalog(){
        double[] values = new double[6];
        values[0] = xbox.getRawAxis(0);
        values[1] = xbox.getRawAxis(1);
        values[2] = xbox.getRawAxis(2);
        values[3] = xbox.getRawAxis(3);
        values[4] = xbox.getRawAxis(4);
        values[5] = xbox.getRawAxis(5);
        return values;
    }
    public static boolean[] getXboxButtons(){
        boolean[] values = new boolean[10];
        values[0] = xbox.getRawButton(0);
        values[1] = xbox.getRawButton(1);
        values[2] = xbox.getRawButton(2);
        values[3] = xbox.getRawButton(3);
        values[4] = xbox.getRawButton(4);
        values[5] = xbox.getRawButton(5);
        values[6] = xbox.getRawButton(6);
        values[7] = xbox.getRawButton(7);
        values[8] = xbox.getRawButton(8);
        values[9] = xbox.getRawButton(9);
        return values;
    }
    //Logitech indexes assigned based on following chart
    //https://ccisdrobonauts.org/uploads/12/8f/128f1b30391ad7a196606e2b57a1a6b1.pdf page 18
    public static double[] getLogiAnalog(){
        double[] values = new double[4];
        values[0] = logi.getRawAxis(0);
        values[1] = logi.getRawAxis(1);
        values[2] = logi.getRawAxis(2);
        values[3] = logi.getRawAxis(3);
        return values;
    }
    public static boolean[] getLogiButtons(){
        boolean[] values = new boolean[10];
        //values[0] = logi.getRawButton(0);
        values[1] = logi.getRawButton(1);
        values[2] = logi.getRawButton(2);
        values[3] = logi.getRawButton(3);
        values[4] = logi.getRawButton(4);
        values[5] = logi.getRawButton(5);
        values[6] = logi.getRawButton(6);
        values[7] = logi.getRawButton(7);
        values[8] = logi.getRawButton(8);
        values[9] = logi.getRawButton(9);
        values[10] = logi.getRawButton(10);
        values[11] = logi.getRawButton(11);
        values[12] = logi.getRawButton(12);
        return values;
    }
}
