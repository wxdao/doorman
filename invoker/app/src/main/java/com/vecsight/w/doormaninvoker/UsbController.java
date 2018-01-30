package com.vecsight.w.doormaninvoker;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class UsbController {
    public UsbManager getUsbManager() {
        return usbManager;
    }

    public void setUsbManager(UsbManager usbManager) {
        this.usbManager = usbManager;
    }

    public UsbDevice getUsbDevice() {
        return usbDevice;
    }

    public void setUsbDevice(UsbDevice usbDevice) {
        this.usbDevice = usbDevice;
    }

    private UsbManager usbManager;
    private UsbDevice usbDevice;

    private static final UsbController ourInstance = new UsbController();

    public static UsbController getInstance() {
        return ourInstance;
    }

    private UsbController() {
    }

    private boolean isReady() {
        return usbDevice != null && usbManager != null;
    }

    public int query(int op, int data) throws UsbControllerDead {
        synchronized (this) {
            if (!isReady()) {
                throw new UsbControllerDead();
            }
            UsbInterface inf = usbDevice.getInterface(0);
            UsbDeviceConnection con = usbManager.openDevice(usbDevice);

            {
                UsbEndpoint wEp = inf.getEndpoint(1);
                ByteBuffer bb = ByteBuffer.allocate(8);
                bb.order(ByteOrder.LITTLE_ENDIAN);
                bb.putInt(op);
                bb.putInt(data);
                byte[] bytes = bb.array();

                con.claimInterface(inf, true);
                int r = con.bulkTransfer(wEp, bytes, bytes.length, 0);
                con.releaseInterface(inf);

                if (r != 8) {
                    throw new UsbControllerDead();
                }
            }

            {
                UsbEndpoint rEp = inf.getEndpoint(0);
                byte[] bytes = new byte[8];

                con.claimInterface(inf, true);
                int r = con.bulkTransfer(rEp, bytes, bytes.length, 0);
                con.releaseInterface(inf);

                if (r != 8) {
                    throw new UsbControllerDead();
                }
                ByteBuffer bb = ByteBuffer.wrap(bytes);
                bb.order(ByteOrder.LITTLE_ENDIAN);
                bb.getInt();
                return bb.getInt();
            }
        }
    }
}

class UsbControllerDead extends Exception {
}
