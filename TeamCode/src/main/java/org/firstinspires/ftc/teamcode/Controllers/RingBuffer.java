package org.firstinspires.ftc.teamcode.Controllers;

import java.lang.reflect.Array;
import java.lang.reflect.Constructor;

/**
this is something that Ben Caunt from Thermal said works better for FTC PID loops
 */
public class RingBuffer<T> {
    private T[] buffer;
    private int pushes = 0;

    private Constructor<? extends T> ctor;

    public RingBuffer(int capacity, Class<T> type) {
        @SuppressWarnings("unchecked")
        final T[] a = (T[]) Array.newInstance(type, capacity);
        this.buffer = a;
    }




    public void push(T value) {
        T[] oldBuffer = buffer;
        for(int i = 0; i < oldBuffer.length-1; i++) {
            if(i == 0) {
                buffer[0] = value;
            } else {
                buffer[i] = oldBuffer[i-1];
            }
        }
        pushes++;
    }

    public T pull() {
        if(pushes < buffer.length) return buffer[pushes-1];
        else return buffer[buffer.length-1];
    }

    public T get(int index) {
        return buffer[index];
    }

    public double avg() {
        double total = 0;
        for(T val : buffer) {
            try{
                total += Double.parseDouble(val.toString());
            } catch (NumberFormatException e) {
                e.printStackTrace();
                return 0;
            }

        }
        double avg = total / buffer.length;
        return avg;
    }
}
