package org.firstinspires.ftc.teamcode.extrautilslib.samples;

import com.icaras84.extrautilslib.core.memsys.EULMemory;

import java.util.List;

public class MemTest {

    public static void main(String[] args) {
        EULMemory.init();
        EULMemory.getMaster().alloc(1);
        EULMemory.getMaster().alloc("Lmao");
        EULMemory.getMaster().alloc("World");
        EULMemory.getMaster().alloc("!");

        printList(EULMemory.getMaster().getTakenHexes());
        System.out.println("\n");

        EULMemory.getMaster().delete(0x000001);

        printList(EULMemory.getMaster().getTakenHexes());
        System.out.println("\n");

        EULMemory.getMaster().alloc(",");

        printList(EULMemory.getMaster().getTakenHexes());
        System.out.println("\n");

        EULMemory.getMaster().delete(0x000003);

        printList(EULMemory.getMaster().getTakenHexes());
        System.out.println("\n");

        EULMemory.getMaster().alloc(".");

        printList(EULMemory.getMaster().getTakenHexes());
        System.out.println("\n");

        System.out.println("\"" + EULMemory.getMaster().getItem(0x000000).get() + "\" Class: " + EULMemory.getMaster().getItem(0x000000).get().getClass());
        System.out.println("\"" + EULMemory.getMaster().getItem(0x000001).get() + "\" Class: " + EULMemory.getMaster().getItem(0x000001).get().getClass());
        System.out.println("\"" + EULMemory.getMaster().getItem(0x000002).get() + "\" Class: " + EULMemory.getMaster().getItem(0x000002).get().getClass());
        System.out.println("\"" + EULMemory.getMaster().getItem(0x000003).get() + "\" Class: " + EULMemory.getMaster().getItem(0x000003).get().getClass());

        EULMemory.getMaster().setItem(0x000000, 3);
        EULMemory.getMaster().setItem(0x000002, -1.4134);

        System.out.println();

        System.out.println("\"" + EULMemory.getMaster().getItem(0x000000).get() + "\" Class: " + EULMemory.getMaster().getItem(0x000000).get().getClass());
        System.out.println("\"" + EULMemory.getMaster().getItem(0x000001).get() + "\" Class: " + EULMemory.getMaster().getItem(0x000001).get().getClass());
        System.out.println("\"" + EULMemory.getMaster().getItem(0x000002).get() + "\" Class: " + EULMemory.getMaster().getItem(0x000002).get().getClass());
        System.out.println("\"" + EULMemory.getMaster().getItem(0x000003).get() + "\" Class: " + EULMemory.getMaster().getItem(0x000003).get().getClass());
    }

    public static <T> void printList(List<T> in){
        String line = "[";
        for (T l: in) {
            line += l.toString() + ", ";
        }
        line += "]";
        line = line.replaceFirst(", ]", "]");

        System.out.println(line);
    }
}
