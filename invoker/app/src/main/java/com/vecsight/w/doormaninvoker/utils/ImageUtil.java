package com.vecsight.w.doormaninvoker.utils;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import java.io.ByteArrayOutputStream;

public class ImageUtil {
    private static final ImageUtil ourInstance = new ImageUtil();

    private ImageUtil() {
    }

    static ImageUtil getInstance() {
        return ourInstance;
    }

    public static byte[] compress(byte[] image, int targetWidth, int targetHeight, int quality) {
        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inJustDecodeBounds = true;
        BitmapFactory.decodeByteArray(image, 0, image.length, options);

        int ratio = 1;
        int width = options.outWidth;
        int height = options.outHeight;
        if (height > targetHeight || width > targetWidth) {
            if (width > height) {
                ratio = width / targetWidth;
            } else {
                ratio = height / targetHeight;
            }
        }

        options.inJustDecodeBounds = false;
        options.inSampleSize = ratio;

        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        BitmapFactory.decodeByteArray(image, 0, image.length, options).compress(Bitmap.CompressFormat.JPEG, quality, outputStream);

        byte[] compressedImage = outputStream.toByteArray();
        return compressedImage;
    }
}
