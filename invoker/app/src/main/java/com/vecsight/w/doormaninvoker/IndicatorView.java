package com.vecsight.w.doormaninvoker;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.graphics.PorterDuffXfermode;
import android.util.AttributeSet;
import android.view.View;

public class IndicatorView extends View {
    private Bitmap bitmap;
    private Paint paintRect = new Paint();
    private Paint paintGreen = new Paint();
    private Paint paintTransparent = new Paint();
    private Paint paintBitmap = new Paint();

    int width, height;

    public IndicatorView(Context context, AttributeSet attrs) {
        super(context, attrs);

        paintRect.setColor(0x000055);
        paintRect.setAlpha(0x55);

        paintGreen.setColor(Color.GREEN);

        paintTransparent.setAlpha(0x00);
        paintTransparent.setXfermode(new PorterDuffXfermode(PorterDuff.Mode.CLEAR));


    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);

        width = this.getMeasuredWidth();
        height = this.getMeasuredHeight();
        bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        Canvas temp = new Canvas(bitmap);
        temp.drawRect(0, 0, width, height, paintRect);
        temp.drawCircle(width / 2, height / 2, Math.min(width, height) / 3 + 10, paintGreen);
        temp.drawCircle(width / 2, height / 2, Math.min(width, height) / 3, paintTransparent);
    }

    protected void onDraw(Canvas canvas) {
        canvas.drawBitmap(bitmap, 0, 0, paintBitmap);
    }
}