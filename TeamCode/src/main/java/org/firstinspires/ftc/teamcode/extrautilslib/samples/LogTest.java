package org.firstinspires.ftc.teamcode.extrautilslib.samples;

import com.icaras84.extrautilslib.core.filehandlers.logger.BasicLogger;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;

public class LogTest {
    public static void main(String[] args) {
        System.setErr(System.out);
        BasicLogger logger = new BasicLogger(new ConsoleHandler(), true);

        logger.log(Level.FINE, "Hello!");
        logger.log(Level.WARNING, "Warning!");

        logger.outputAll();
    }
}
