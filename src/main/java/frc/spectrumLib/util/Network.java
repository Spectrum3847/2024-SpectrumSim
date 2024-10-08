package frc.spectrumLib.util;

import java.net.*;

/** Common Network Utilties */
public class Network {

    /**
     * Gets the MAC address of the robot
     *
     * @return the MAC address of the robot
     */
    public static String getMACaddress() {
        InetAddress localHost;
        NetworkInterface ni;
        byte[] hardwareAddress;
        String MAC = "";
        for (int i = 0; i < 10; i++) {
            try {
                localHost = InetAddress.getLocalHost();
                if (localHost == null) return "UNKNOWN";
                ni = NetworkInterface.getByInetAddress(localHost);
                if (ni == null) return "UNKNOWN";
                hardwareAddress = ni.getHardwareAddress();
                if (hardwareAddress == null) return "UNKNOWN";

                String[] hexadecimal = new String[hardwareAddress.length];
                for (int j = 0; j < hardwareAddress.length; j++) {
                    hexadecimal[j] = String.format("%02X", hardwareAddress[j]);
                }
                MAC = String.join(":", hexadecimal);
                return MAC;
            } catch (UnknownHostException | SocketException e) {
            }
        }
        return "UNKNOWN";
    }

    /**
     * Gets the IP address of the robot
     *
     * @return the IP address of the robot
     */
    public static String getIPaddress() {
        InetAddress localHost;
        String IP = "";
        for (int i = 0; i < 10; i++) {
            try {
                localHost = InetAddress.getLocalHost();
                IP = localHost.getHostAddress();
                return IP;
            } catch (UnknownHostException e) {
            }
        }
        return "UNKNOWN";
    }

    /**
     * Gets the IP Address of the device at the address such as "limelight.local"
     *
     * @return the IP Address of the device
     */
    public static String getIPaddress(String deviceNameAddress) {
        InetAddress localHost;
        String IP = "";
        for (int i = 0; i < 10; i++) {
            try {
                localHost = InetAddress.getByName(deviceNameAddress);
                IP = localHost.getHostAddress();
                return IP;
            } catch (UnknownHostException e) {
            }
        }
        return "UNKNOWN";
    }
}
