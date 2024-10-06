package frc.spectrumLib.util;

// Based on 254-2023 Class
// https://github.com/Team254/FRC-2023-Public/blob/main/src/main/java/com/team254/lib/drivers/CanDeviceId.java
public class CanDeviceId {
    private final int deviceNumber;
    private final String mBus;

    public CanDeviceId(int deviceNumber, String bus) {
        this.deviceNumber = deviceNumber;
        mBus = bus;
    }

    // Use the default bus name (empty string).
    public CanDeviceId(int deviceNumber) {
        this(deviceNumber, "");
    }

    public int getDeviceNumber() {
        return deviceNumber;
    }

    public String getBus() {
        return mBus;
    }

    public boolean equals(CanDeviceId other) {
        return other.deviceNumber == deviceNumber && other.mBus == mBus;
    }
}
