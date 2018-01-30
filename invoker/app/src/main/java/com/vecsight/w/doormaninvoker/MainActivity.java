package com.vecsight.w.doormaninvoker;

import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.PointF;
import android.graphics.Rect;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.media.FaceDetector;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.util.Pair;
import android.widget.Toast;

import com.flurgle.camerakit.CameraListener;
import com.flurgle.camerakit.CameraView;
import com.vecsight.w.doormaninvoker.utils.ImageUtil;

import java.io.ByteArrayOutputStream;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class MainActivity extends AppCompatActivity {
    private CameraView cameraView;
    private FaceWorker faceWorker;

    private boolean forceSearch = false;
    private int preFound = 0;

    private ScheduledExecutorService scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();
    private ScheduledFuture<?> taskHandle;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        UsbController usbController = UsbController.getInstance();
        usbController.setUsbManager((UsbManager) getSystemService(Context.USB_SERVICE));

        Intent intent = getIntent();
        String action = intent.getAction();

        if ("android.hardware.usb.action.USB_DEVICE_ATTACHED".equals(action)) {
            UsbDevice usbDevice = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            if (usbDevice != null) {
                Toast.makeText(this, "usb connected", Toast.LENGTH_LONG).show();
                usbController.setUsbDevice(usbDevice);
            }
        }

        faceWorker = new FaceWorker(this, new Handler());

        cameraView = findViewById(R.id.cameraView);
        cameraView.setCameraListener(new CameraListener() {
            @Override
            public void onPictureTaken(byte[] picture) {
                super.onPictureTaken(picture);

                byte[] compressed = ImageUtil.compress(picture, 512, 512, 30);
                Bitmap bitmap = BitmapFactory.decodeByteArray(compressed, 0, compressed.length);
                Bitmap compressedBitmap = bitmap.copy(Bitmap.Config.RGB_565, true);

                int width = compressedBitmap.getWidth();
                int height = compressedBitmap.getHeight();

                FaceDetector faceDetector = new FaceDetector(width, height, 5);
                FaceDetector.Face[] faces = new FaceDetector.Face[5];
                int faceCount = faceDetector.findFaces(compressedBitmap, faces);

                int radius = Math.min(width, height) / 3;
                Rect boundingRect = new Rect(width / 2 - radius, height / 2 - radius, width / 2 + radius, height / 2 + radius);

                int found = -1;
                for (int i = 0; i < faceCount; i++) {
                    PointF midPoint = new PointF();
                    faces[i].getMidPoint(midPoint);
                    float eyeDistance = faces[i].eyesDistance();

                    Rect faceRect = new Rect((int) (midPoint.x - eyeDistance), (int) (midPoint.y - eyeDistance), (int) (midPoint.x + eyeDistance), (int) (midPoint.y + eyeDistance));
                    if (boundingRect.contains(faceRect) && eyeDistance >= radius / 3) {
                        found = i;
                    }
                }

                if (forceSearch) {
                    Bitmap croppedBitmap = Bitmap.createBitmap(bitmap, boundingRect.left, boundingRect.top, boundingRect.width(), boundingRect.height());
                    ByteArrayOutputStream stream = new ByteArrayOutputStream();
                    croppedBitmap.compress(Bitmap.CompressFormat.JPEG, 100, stream);
                    byte[] croppedPicture = stream.toByteArray();
                    faceWorker.pushImage(croppedPicture, new Pair<>("search", ""));
                    forceSearch = false;
                } else if (found != -1) {
                    if (preFound == 1) {
                        Bitmap croppedBitmap = Bitmap.createBitmap(bitmap, boundingRect.left, boundingRect.top, boundingRect.width(), boundingRect.height());
                        ByteArrayOutputStream stream = new ByteArrayOutputStream();
                        croppedBitmap.compress(Bitmap.CompressFormat.JPEG, 100, stream);
                        byte[] croppedPicture = stream.toByteArray();
                        faceWorker.pushImage(croppedPicture, new Pair<>("search", ""));
                    }
                    preFound++;
                } else {
                    preFound = 0;
                }
            }
        });
    }

    @Override
    protected void onResume() {
        super.onResume();
        cameraView.start();
        faceWorker.resumeWorker();
        final Handler handler = new Handler();
        taskHandle = scheduledExecutorService.scheduleWithFixedDelay(new Runnable() {
            @Override
            public void run() {
                handler.post(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            synchronized (MainActivity.this) {
                                cameraView.captureImage();
                            }
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    }
                });
            }
        }, 500, 500, TimeUnit.MILLISECONDS);
    }

    @Override
    protected void onPause() {
        cameraView.stop();
        if (taskHandle != null) {
            taskHandle.cancel(true);
            taskHandle = null;
        }
        super.onPause();
    }
}
