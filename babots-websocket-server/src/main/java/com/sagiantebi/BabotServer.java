/*
 * BabotServer.java
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

package com.sagiantebi;

import io.vertx.core.AsyncResult;
import io.vertx.core.Handler;
import io.vertx.core.Vertx;
import io.vertx.core.buffer.Buffer;
import io.vertx.core.http.HttpServer;
import io.vertx.core.http.HttpServerOptions;
import io.vertx.core.http.HttpServerRequest;
import io.vertx.core.http.ServerWebSocket;

import java.util.ArrayList;
import java.util.List;

public class BabotServer {

    private static List<ServerWebSocket> serverWebSocketList = new ArrayList<>();

    public static void main(String[] args) {
        Vertx vertx = Vertx.vertx();
        HttpServerOptions options = new HttpServerOptions();
        options.setLogActivity(true);
        HttpServer httpServer = vertx.createHttpServer(options);
        httpServer.requestHandler(event -> handleRequestEvent(event)).listen(8080, event -> {});
    }

    /**
     *
     * @param angle the angle recieved from the joystick
     * @param p the power received from the joystick, i.e. how far the finger was from the center of the joystick.
     * @return a String which the babot can read and move accordingly.
     */
    private static String getEnginePower(int angle, int p) {
        //from 90 to 270 it's the right motor
        //from 270 to 90 it's the left motor
        //clockwise
        //90 = f9f9
        //0 = s0f9
        //270 = r9r9
        //180 = f9s0

        int rp = 0;
        int lp = 0;
        //int angle = 90;

        float power = ((float)p / 100f);

        boolean reverse = false;
        if (angle <= 180 && angle >= 0) {
            //forward on both engines.
            if (angle >= 90) {
                //power to the left engine
                rp = (90 - (angle-90)) / 10; //.i.e 120, (90-30) = 60, which means f6
                lp = 9; //this one has all the power
            } else {
                rp = 9;
                lp = angle / 10; //i.e. 60, the left engine is at power 6
            }
        } else {
            reverse = true;
            //reverse on both engines.
            //360 is r power to left
            if (angle >= 270) {
                rp = (360-angle)/ 10;//300 (360-300) = 60, 6
                lp = 9;
            } else {
                rp = 9;
                lp = (90 - (270-angle))/10; //200 (270-200)
            }
            int t = rp;
            rp = lp;
            lp = t;
        }

        lp = (int) (lp*power);
        rp = (int) (rp*power);

        if (p < 10) {
            return "s0s0";
        }

        return String.format(reverse ? "r%dr%d" : "f%df%d", lp, rp);
    }

    private static void handleRequestEvent(HttpServerRequest request) {
        if (request.path().equals("/ctlm") || request.path().equals("/ctl") || request.path().equals("/babot")) {
            ServerWebSocket webSocket = request.upgrade();
            webSocket.closeHandler(new Handler<Void>() {
                @Override
                public void handle(Void event) {
                    serverWebSocketList.remove(webSocket);
                }
            });
            webSocket.handler(new Handler<Buffer>() {
                @Override
                public void handle(Buffer event) {
                    if (webSocket.path().equals("/ctl")) {
                        for (ServerWebSocket socket : serverWebSocketList) {
                            socket.writeTextMessage(event.toString());
                        }
                    } else if (webSocket.path().equals("/ctlm")) {
                        String angleAndPower = event.toString();
                        String[] parts = angleAndPower.split("-");
                        String ep = getEnginePower(Integer.parseInt(parts[0]), Integer.parseInt(parts[1]));
                        for (ServerWebSocket socket : serverWebSocketList) {
                            socket.writeTextMessage(ep);
                        }
                    }
                }
            });
            if (request.path().equals("/babot")) {
                serverWebSocketList.add(webSocket);
            }
        } else {
            request.response().putHeader("Content-Type", "Text/HTML").end("<div>Nothing to see here - yet!</div>");
        }
    }
}
