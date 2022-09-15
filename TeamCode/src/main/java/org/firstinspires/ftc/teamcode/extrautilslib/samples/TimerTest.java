package org.firstinspires.ftc.teamcode.extrautilslib.samples;

import com.icaras84.extrautilslib.core.timer.EULClock;

public class TimerTest {

    public static void main(String[] args) {
        //firstTest();
        secondTest();
    }

    public static void firstTest(){
        EULClock timer = new EULClock();

        long start = System.currentTimeMillis();
        System.out.println("Milliseconds: " + (System.currentTimeMillis() - start) + " ± 1ms");
        timer.waitFor(1111);
        System.out.println("Milliseconds: " + (System.currentTimeMillis() - start) + " ± 1ms");
    }

    public static void secondTest(){
        EULClock timer = new EULClock();

        timer.start();
        for (int i = 0; i < 100; i++) {
            System.out.append("*Tick* Time Elapsed: " + (timer.getElapsedTime() * EULClock.NANO2MS) + "ms (± 1 ms)\n");
            timer.waitFor(10);
        }
        timer.stop();
        System.out.flush();
    }
}
