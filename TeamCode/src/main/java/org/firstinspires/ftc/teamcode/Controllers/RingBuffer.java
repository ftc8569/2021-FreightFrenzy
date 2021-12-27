package org.firstinspires.ftc.teamcode.Controllers;

import java.lang.reflect.Array;

/**
this is something that Ben Caunt from Thermal said works better for FTC PID loops
 */
public class RingBuffer {
    private double[] buffer;
    private int pushes = 0;

    public RingBuffer(int capacity) {
        buffer = new double[capacity];
    }

    public void push(double value) {
        double[] oldBuffer = buffer;
        for(int i = 0; i < oldBuffer.length-1; i++) {
            if(i == 0) {
                buffer[0] = value;
            } else {
                buffer[i] = oldBuffer[i-1];
            }
        }
        pushes++;
    }

    public double pull() {
        if(pushes < buffer.length) return buffer[pushes-1];
        else return buffer[buffer.length-1];
    }

    public double avg() {
        double total = 0;
        for(double val : buffer) {
            total += val;
        }
        double avg = total / buffer.length;
        return avg;
    }
}
