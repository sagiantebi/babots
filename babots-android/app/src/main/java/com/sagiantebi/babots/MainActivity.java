/*
 * MainActivity.java
 *
 * Copyright 2017 Sagi Antebi
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package com.sagiantebi.babots;

import android.os.Handler;
import android.os.Looper;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;

import com.koushikdutta.async.callback.CompletedCallback;
import com.koushikdutta.async.http.AsyncHttpClient;
import com.koushikdutta.async.http.WebSocket;

import io.github.controlwear.virtual.joystick.android.JoystickView;

public class MainActivity extends AppCompatActivity {

    private JoystickView mJoystickView;
    private Handler mHandlerMain;
    private WebSocket mActiveWebSocket;
    private Button mButton;
    private MovementVector mMovementVector = new MovementVector();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        mHandlerMain = new Handler(Looper.getMainLooper());
        mJoystickView = (JoystickView) findViewById(R.id.joystickView);
        mButton = (Button) findViewById(R.id.button);
        mButton.setOnClickListener(mOnClickListener);
        mJoystickView.setOnMoveListener(new JoystickView.OnMoveListener() {
            @Override
            public void onMove(int angle, int strength) {
                mMovementVector.mAngle = angle;
                mMovementVector.mStrength = strength;
            }
        });
        connect();
    }


    private Runnable mSendLoopRunnable = new Runnable() {

        private MovementVector mLastMovementVector = new MovementVector();

        @Override
        public void run() {
            boolean dirty = false;
            if (!mLastMovementVector.equals(mMovementVector)) {
                mLastMovementVector.setTo(mMovementVector);
                dirty = true;
            }
            if (isValidAndConnected()) {
                if (dirty) mActiveWebSocket.send(String.format("%d-%d", mLastMovementVector.mAngle, mLastMovementVector.mStrength));
                mHandlerMain.postDelayed(this, 10);
            }
        }
    };


    private boolean isValidAndConnected() {
        return !isFinishing() && mActiveWebSocket != null;
    }

    private void connect() {

        if (mActiveWebSocket != null) {
            mActiveWebSocket.close();
        }

        AsyncHttpClient.getDefaultInstance().websocket("http://host:8080/ctlm", null, new AsyncHttpClient.WebSocketConnectCallback() {
            @Override
            public void onCompleted(Exception ex, WebSocket webSocket) {
                if (ex != null) {
                    Log.e(this.getClass().getName(), "Failed to connect", ex);
                } else {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mButton.setText("Disconnect");
                        }
                    });
                    mActiveWebSocket = webSocket;
                    mHandlerMain.post(mSendLoopRunnable);
                    webSocket.setClosedCallback(new CompletedCallback() {
                        @Override
                        public void onCompleted(Exception ex) {
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    mButton.setText("Connect");
                                }
                            });

                            mActiveWebSocket = null;
                            mHandlerMain.removeCallbacksAndMessages(null);
                        }
                    });
                }
            }
        });
    }

    private View.OnClickListener mOnClickListener = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            if (v == mButton) {
                if (mActiveWebSocket == null) {
                    connect();
                } else {
                    mActiveWebSocket.close();
                }
            }
        }
    };

    @Overrive
    public void onDestroy() {
        super.onDestroy();
        if (mHandlerMain != null) {
            mHandlerMain.removeCallbacksAndMessages(null);
        }
        if (mActiveWebSocket != null) {
            mActiveWebSocket.close();
        }
    }

    private static class MovementVector {
        int mAngle;
        int mStrength;


        public void setTo(MovementVector other) {
            mAngle = other.mAngle;
            mStrength = other.mStrength;
        }


        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;

            MovementVector that = (MovementVector) o;

            if (mAngle != that.mAngle) return false;
            return mStrength == that.mStrength;

        }

        @Override
        public int hashCode() {
            int result = mAngle;
            result = 31 * result + mStrength;
            return result;
        }
    }


}
