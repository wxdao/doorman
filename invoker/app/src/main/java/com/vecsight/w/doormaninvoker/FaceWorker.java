package com.vecsight.w.doormaninvoker;

import android.content.Context;
import android.os.Handler;
import android.support.v7.app.AlertDialog;
import android.util.Base64;
import android.util.Log;
import android.util.Pair;
import android.widget.Toast;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;

import java.util.Locale;
import java.util.concurrent.LinkedBlockingQueue;

import okhttp3.MediaType;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;

class FaceWorker extends Thread {
    private Context context;
    private Handler handler;

    private Toast mToast = null;

    private LinkedBlockingQueue<Pair<byte[], Pair<String, String>>> queue = new LinkedBlockingQueue<>();

    private Gson gson = new GsonBuilder().create();

    private OkHttpClient client = new OkHttpClient();

    UsbController usbController = UsbController.getInstance();

    private boolean exit = true;

    FaceWorker(Context context, Handler handler) {
        this.handler = handler;
        this.context = context;

        mToast = new Toast(context);

        exit = false;
        this.start();
    }

    void pushImage(byte[] image, Pair<String, String> opt) {
        try {
            queue.put(new Pair<>(image, opt));
        } catch (InterruptedException ignored) {
        }
    }

    void resumeWorker() {
        if (!this.isAlive()) {
            exit = false;
            this.start();
        }
    }

    private void toast(final String text) {
        handler.post(new Runnable() {
            @Override
            public void run() {
                if (mToast != null) {
                    mToast.cancel();
                }
                mToast = Toast.makeText(context, text, Toast.LENGTH_LONG);
                mToast.show();
            }
        });
    }

    private void dialog(final String text) {
        handler.post(new Runnable() {
            @Override
            public void run() {
                new AlertDialog.Builder(context).setTitle("Result").setMessage(text).show();
            }
        });
    }

    private Pair<String, Integer> search(byte[] image, int controllerID, int token) throws Exception {
        JsonObject jsonRequest = new JsonObject();
        jsonRequest.addProperty("image", Base64.encodeToString(image, Base64.NO_WRAP));
        jsonRequest.addProperty("token", token & 0xffffffffL);
        RequestBody requestBody = RequestBody.create(
                MediaType.parse("application/json; charset=utf-8"),
                gson.toJson(jsonRequest)
        );
        Request request = new Request.Builder()
                .url(String.format(Locale.CHINA, "http://192.168.1.110:3389/controller/%d/sign", controllerID & 0xffffffffL))
                .post(requestBody)
                .build();
        okhttp3.Response response = client.newCall(request).execute();
        if (response.code() != 200) {
            throw new Exception(response.body().string());
        }
        JsonObject rep = gson.fromJson(response.body().string(), JsonObject.class);
        return new Pair<>(rep.get("name").getAsString(), (int) rep.get("sign").getAsLong());
    }

    @Override
    public void run() {
        Log.i("FaceWorker", "thread started");

        while (!exit) {
            try {
                Pair<byte[], Pair<String, String>> imagePair = queue.take();
                byte[] image = imagePair.first;
                toast("processing w/ size: " + String.valueOf(image.length));
                switch (imagePair.second.first) {
                    case "search": {
                        Pair<String, Integer> res = search(image, usbController.query(0, 0), usbController.query(1, 0));
                        toast("Welcome(" + String.valueOf(usbController.query(2, res.second)) + "): " + res.first);
                        break;
                    }
                    default:
                        toast("unknown operation");
                        continue;
                }
            } catch (InterruptedException e) {
                break;
            } catch (Exception e) {
                toast(e.toString());
                e.printStackTrace();
            }
        }
        Log.i("FaceWorker", "thread exited");
    }
}
