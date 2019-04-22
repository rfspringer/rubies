package org.firstinspires.ftc.teamcode.Library;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Created by FTC 9890 on 7/26/2017.
 *
 * In your program, declare an instance of the object.
 * You can pass an optional file name, the default is "log"
 *
 * Ex.
 * FTCLogger logger = new FTCLogger();
 * FTCLogger logger = new FTCLogger("myLog");
 *
 *
 * Whenever you want to log data, use the writeLine() method.
 * This method take a list of objects. You can pass it strings, integers, doubles, arrays or any combination. You can also pass it an array of data.
 *
 * Ex.
 * logger.writeLine("test", gyro.getYaw(), motor.getCurrentPosition());
 * logger.writeLine(myArray);
 *
 * In your stop() method, call the closeFile() method.
 *
 * If you have any questions, contact us at ftcrubies@gmail.com.
 * We'd be happy to help!
 */

public class FTCLogger {
    //Declares PrintWriter and FileWriter variables
    PrintWriter pw;
    FileWriter fw;

    public FTCLogger(String logIdentifier) {
        //Declare a log directory
        File logDir = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/LogDir");

        if (!logDir.exists()){
            logDir.mkdir();     //If there isn't a LogDir folder, create one
        }

        try {
            String fileName = logIdentifier + new SimpleDateFormat("_yyyyMMdd_HHmmss").format(new Date()) + ".csv";     //Declare the file name with the log identifier (ex. program name), then date

            fw = new FileWriter(Environment.getExternalStorageDirectory().getAbsolutePath() + "/LogDir/" + fileName);       //Create a file in the phone's external storage under the LogDir folder
            pw = new PrintWriter(fw, false);
        }
        catch(IOException ioe){
        }
    }

    public FTCLogger() {
        //If no log identifier is given, use "log" as default
        this("log");
    }

    public void writeLine(Object... logData){
        //Create and print a timestamp for the log entry
        String timestamp = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss.SSS").format(new Date());
        pw.print(timestamp);

        for (Object logEntry : logData){
            //Separate previous value from current value with a comma, log each entry
            pw.print(",");
            pw.print(logEntry.toString());
        }

        //End the line after all entries in the passed data have been added to the file
        pw.println();
    }

    public void closeFile() {
        //Close the file
        pw.close();
        try {
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
