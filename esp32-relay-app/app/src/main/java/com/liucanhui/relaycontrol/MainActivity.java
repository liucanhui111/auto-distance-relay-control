package com.liucanhui.relaycontrol;

import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.View;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.HttpURLConnection;
import java.net.InetAddress;
import java.net.URL;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MainActivity extends AppCompatActivity {

    private static final String PREFS = "relay_prefs";
    private static final String KEY_DEVICE_IP = "device_ip";

    private TextView deviceStatus, relayStatus, logText;
    private TextView distanceValue, distanceLabel, modeHint;
    private TextView offThresholdLabel, onThresholdLabel;
    private View controlPanel, progressBar, thresholdPanel;
    private SeekBar offThresholdSeek, onThresholdSeek;
    private Button btnOn, btnOff, btnToggle, btnProvision, btnSearch, btnManual, btnAuto;
    private String deviceIp = null;
    private boolean autoMode = false;
    private double offThreshold = 2.5;
    private double onThreshold = 10.0;
    private final Handler thresholdHandler = new Handler(Looper.getMainLooper());
    private Runnable pendingThresholdSend = null;

    private final ExecutorService executor = Executors.newFixedThreadPool(4);
    private final Handler mainHandler = new Handler(Looper.getMainLooper());

    private static final int DISCOVERY_PORT = 18080;
    private static final String DISCOVERY_MAGIC = "RELAY_DISCOVER";
    private boolean polling = false;
    private boolean distancePolling = false;
    private boolean returningFromProvision = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        deviceStatus = findViewById(R.id.deviceStatus);
        relayStatus = findViewById(R.id.relayStatus);
        logText = findViewById(R.id.logText);
        controlPanel = findViewById(R.id.controlPanel);
        distanceValue = findViewById(R.id.distanceValue);
        distanceLabel = findViewById(R.id.distanceLabel);
        modeHint = findViewById(R.id.modeHint);
        progressBar = findViewById(R.id.progressBar);
        btnOn = findViewById(R.id.btnOn);
        btnOff = findViewById(R.id.btnOff);
        btnToggle = findViewById(R.id.btnToggle);
        btnProvision = findViewById(R.id.btnProvision);
        btnSearch = findViewById(R.id.btnSearch);
        btnManual = findViewById(R.id.btnManual);
        btnAuto = findViewById(R.id.btnAuto);

        thresholdPanel = findViewById(R.id.thresholdPanel);
        offThresholdLabel = findViewById(R.id.offThresholdLabel);
        onThresholdLabel = findViewById(R.id.onThresholdLabel);
        offThresholdSeek = findViewById(R.id.offThresholdSeek);
        onThresholdSeek = findViewById(R.id.onThresholdSeek);

        /* 断开距离滑块: 2.5-6.0cm, progress 0-35, 步长0.1 */
        offThresholdSeek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                offThreshold = 2.5 + progress * 0.1;
                offThresholdLabel.setText(String.format(Locale.getDefault(), "%.1f cm", offThreshold));
            }
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                sendThreshold();
            }
        });

        /* 吸合距离滑块: 10.0-15.0cm, progress 0-50, 步长0.1 */
        onThresholdSeek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                onThreshold = 10.0 + progress * 0.1;
                onThresholdLabel.setText(String.format(Locale.getDefault(), "%.1f cm", onThreshold));
            }
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                sendThreshold();
            }
        });

        btnOn.setOnClickListener(v -> sendCmd("1", "吸合 ON"));
        btnOff.setOnClickListener(v -> sendCmd("0", "断开 OFF"));
        btnToggle.setOnClickListener(v -> sendCmd("T", "切换电平"));
        btnProvision.setOnClickListener(v -> {
            returningFromProvision = true;
            startActivity(new Intent(this, ProvisionActivity.class));
        });
        btnSearch.setOnClickListener(v -> discoverDevice());

        btnManual.setOnClickListener(v -> setMode(false));
        btnAuto.setOnClickListener(v -> setMode(true));

        controlPanel.setVisibility(View.VISIBLE);

        SharedPreferences sp = getSharedPreferences(PREFS, MODE_PRIVATE);
        deviceIp = sp.getString(KEY_DEVICE_IP, null);
        if (deviceIp != null) {
            log("已保存设备IP: " + deviceIp);
            checkDevice();
        } else {
            log("未发现设备，正在搜索局域网...");
            discoverDevice();
        }
    }

    private void checkDevice() {
        if (deviceIp == null) return;
        executor.execute(() -> {
            try {
                URL url = new URL("http://" + deviceIp + "/status");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("GET");
                conn.setConnectTimeout(3000);
                conn.setReadTimeout(3000);
                conn.connect();
                if (conn.getResponseCode() == 200) {
                    mainHandler.post(() -> {
                        deviceStatus.setText("已连接 · " + deviceIp);
                        controlPanel.setVisibility(View.VISIBLE);
                    });
                    startPolling();
                    startDistancePolling();
                } else {
                    mainHandler.post(() -> {
                        log("保存的IP不在线，自动搜索设备...");
                        discoverDevice();
                    });
                }
                conn.disconnect();
            } catch (Exception e) {
                mainHandler.post(() -> {
                    log("保存的IP连接失败，自动搜索设备...");
                    discoverDevice();
                });
            }
        });
    }

    private void saveDeviceIp(String ip) {
        deviceIp = ip;
        SharedPreferences sp = getSharedPreferences(PREFS, MODE_PRIVATE);
        sp.edit().putString(KEY_DEVICE_IP, ip).apply();
        log("设备IP已保存: " + ip);
    }

    private void discoverDevice() {
        mainHandler.post(() -> {
            btnSearch.setEnabled(false);
            deviceStatus.setText("搜索中...");
        });
        executor.execute(() -> {
            DatagramSocket socket = null;
            try {
                socket = new DatagramSocket();
                socket.setBroadcast(true);
                socket.setSoTimeout(5000);

                byte[] sendData = DISCOVERY_MAGIC.getBytes();
                DatagramPacket sendPacket = new DatagramPacket(
                        sendData, sendData.length,
                        InetAddress.getByName("255.255.255.255"), DISCOVERY_PORT);
                socket.send(sendPacket);
                log("已发送广播搜索...");

                byte[] recvBuf = new byte[256];
                DatagramPacket recvPacket = new DatagramPacket(recvBuf, recvBuf.length);
                socket.receive(recvPacket);

                String resp = new String(recvPacket.getData(), 0, recvPacket.getLength());
                String hostIp = recvPacket.getAddress().getHostAddress();
                log("发现设备: " + resp);

                mainHandler.post(() -> {
                    saveDeviceIp(hostIp);
                    deviceStatus.setText("已连接 · " + hostIp);
                    controlPanel.setVisibility(View.VISIBLE);
                    btnSearch.setEnabled(true);
                    Toast.makeText(this, "发现设备: " + hostIp, Toast.LENGTH_SHORT).show();
                });
                startPolling();
                startDistancePolling();
            } catch (Exception e) {
                mainHandler.post(() -> {
                    log("搜索失败: " + e.getMessage());
                    deviceStatus.setText("未发现设备，请点击配网");
                    btnSearch.setEnabled(true);
                    Toast.makeText(this, "未发现设备，请先配网", Toast.LENGTH_LONG).show();
                });
            } finally {
                if (socket != null) socket.close();
            }
        });
    }

    private void startPolling() {
        if (polling) return;
        polling = true;
        pollStatus();
    }

    private void pollStatus() {
        if (!polling || deviceIp == null) return;
        executor.execute(() -> {
            try {
                URL url = new URL("http://" + deviceIp + "/status");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("GET");
                conn.setConnectTimeout(3000);
                conn.setReadTimeout(3000);
                conn.connect();
                if (conn.getResponseCode() == 200) {
                    BufferedReader r = new BufferedReader(new InputStreamReader(conn.getInputStream()));
                    String body = r.readLine();
                    r.close();
                    conn.disconnect();

                    JSONObject json = new JSONObject(body);
                    boolean on = json.optInt("relay_on", 0) == 1;
                    mainHandler.post(() -> updateRelayDisplay(on));
                } else {
                    conn.disconnect();
                }
            } catch (Exception e) {
                /* 静默失败 */
            }
            mainHandler.postDelayed(this::pollStatus, 2000);
        });
    }

    /* ========== 距离轮询 ========== */
    private void startDistancePolling() {
        if (distancePolling) return;
        distancePolling = true;
        pollDistance();
    }

    private void pollDistance() {
        if (!distancePolling || deviceIp == null) return;
        executor.execute(() -> {
            try {
                URL url = new URL("http://" + deviceIp + "/distance");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("GET");
                conn.setConnectTimeout(3000);
                conn.setReadTimeout(3000);
                conn.connect();
                if (conn.getResponseCode() == 200) {
                    BufferedReader r = new BufferedReader(new InputStreamReader(conn.getInputStream()));
                    String body = r.readLine();
                    r.close();
                    conn.disconnect();

                    JSONObject json = new JSONObject(body);
                    double distance = json.optDouble("distance", -1);
                    String mode = json.optString("mode", "manual");
                    boolean relayOn = json.optInt("relay_on", 0) == 1;
                    double srvOff = json.optDouble("auto_off", 2.5);
                    double srvOn = json.optDouble("auto_on", 10.0);

                    mainHandler.post(() -> {
                        /* 同步滑块位置（不触发回调） */
                        if (Math.abs(srvOff - offThreshold) > 0.05) {
                            offThreshold = srvOff;
                            offThresholdSeek.setProgress((int)Math.round((srvOff - 2.5) * 10));
                            offThresholdLabel.setText(String.format(Locale.getDefault(), "%.1f cm", srvOff));
                        }
                        if (Math.abs(srvOn - onThreshold) > 0.05) {
                            onThreshold = srvOn;
                            onThresholdSeek.setProgress((int)Math.round((srvOn - 10.0) * 10));
                            onThresholdLabel.setText(String.format(Locale.getDefault(), "%.1f cm", srvOn));
                        }
                        updateDistanceUI(distance, mode, relayOn);
                    });
                } else {
                    conn.disconnect();
                }
            } catch (Exception e) {
                /* 静默失败 */
            }
            mainHandler.postDelayed(this::pollDistance, 500);
        });
    }

    private void updateDistanceUI(double distance, String mode, boolean relayOn) {
        /* 更新模式状态 */
        boolean newAutoMode = mode.equals("auto");
        if (newAutoMode != autoMode) {
            autoMode = newAutoMode;
            updateModeButtons();
        }

        /* 更新距离显示 */
        if (distance < 0) {
            distanceValue.setText("--");
            distanceLabel.setText("传感器未就绪");
            distanceLabel.setTextColor(getColor(R.color.text_muted));
            progressBar.setVisibility(View.GONE);
        } else {
            distanceValue.setText(String.format(Locale.getDefault(), "%.1f", distance));

            FrameLayout track = (FrameLayout) progressBar.getParent();
            int trackWidth = track.getWidth();
            if (trackWidth > 0) {
                int progressWidth;
                double maxDist = 20.0;
                if (distance > maxDist) distance = maxDist;
                progressWidth = (int) (trackWidth * (distance / maxDist));

                FrameLayout.LayoutParams lp = new FrameLayout.LayoutParams(
                        progressWidth, FrameLayout.LayoutParams.MATCH_PARENT);
                progressBar.setLayoutParams(lp);
                progressBar.setVisibility(View.VISIBLE);

                /* 颜色: <2.5cm红色, 2.5-10cm黄色, >10cm绿色 */
                if (distance < 2.5) {
                    progressBar.setBackgroundResource(R.drawable.progress_red_bg);
                    distanceLabel.setText("危险距离 · 继电器断开");
                    distanceLabel.setTextColor(getColor(R.color.danger));
                } else if (distance > 10.0) {
                    progressBar.setBackgroundResource(R.drawable.progress_green_bg);
                    distanceLabel.setText("安全距离 · 继电器吸合");
                    distanceLabel.setTextColor(getColor(R.color.success));
                } else {
                    progressBar.setBackgroundResource(R.drawable.progress_yellow_bg);
                    distanceLabel.setText("过渡区间 · 等待");
                    distanceLabel.setTextColor(0xFFFFA502);
                }
            }
        }

        /* 更新继电器状态显示 */
        updateRelayDisplay(relayOn);
    }

    private void updateModeButtons() {
        if (autoMode) {
            btnManual.setTextColor(getColor(R.color.text_secondary));
            btnManual.setBackgroundResource(R.drawable.btn_toggle_bg);
            btnAuto.setTextColor(getColor(R.color.white));
            btnAuto.setBackgroundResource(R.drawable.btn_accent_bg);
            modeHint.setText("自动模式: 滑动设置阈值，自动控制继电器");
            thresholdPanel.setVisibility(View.VISIBLE);
            btnOn.setEnabled(false);
            btnOff.setEnabled(false);
            btnToggle.setEnabled(false);
            btnOn.setAlpha(0.4f);
            btnOff.setAlpha(0.4f);
            btnToggle.setAlpha(0.4f);
        } else {
            btnManual.setTextColor(getColor(R.color.white));
            btnManual.setBackgroundResource(R.drawable.btn_accent_bg);
            btnAuto.setTextColor(getColor(R.color.text_secondary));
            btnAuto.setBackgroundResource(R.drawable.btn_toggle_bg);
            modeHint.setText("手动模式: 手动点击按钮控制继电器");
            thresholdPanel.setVisibility(View.GONE);
            btnOn.setEnabled(true);
            btnOff.setEnabled(true);
            btnToggle.setEnabled(true);
            btnOn.setAlpha(1.0f);
            btnOff.setAlpha(1.0f);
            btnToggle.setAlpha(1.0f);
        }
    }

    /* 发送阈值到 S3 */
    private void sendThreshold() {
        if (deviceIp == null) return;
        executor.execute(() -> {
            try {
                URL url = new URL("http://" + deviceIp + "/threshold?off=" + offThreshold + "&on=" + onThreshold);
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("GET");
                conn.setConnectTimeout(3000);
                conn.setReadTimeout(3000);
                conn.connect();
                int code = conn.getResponseCode();
                BufferedReader r = new BufferedReader(new InputStreamReader(conn.getInputStream()));
                String resp = r.readLine();
                r.close();
                conn.disconnect();
                if (code == 200 && resp != null && resp.contains("OK")) {
                    mainHandler.post(() -> log("阈值更新: 断开<" + String.format(Locale.getDefault(), "%.1f", offThreshold) + "cm, 吸合>" + String.format(Locale.getDefault(), "%.1f", onThreshold) + "cm"));
                }
            } catch (Exception e) {
                mainHandler.post(() -> log("阈值更新失败: " + e.getMessage()));
            }
        });
    }

    private void setMode(boolean auto) {
        if (deviceIp == null) {
            Toast.makeText(this, "请先添加设备", Toast.LENGTH_SHORT).show();
            return;
        }
        String modeStr = auto ? "auto" : "manual";
        String body = "mode=" + modeStr;
        executor.execute(() -> {
            try {
                URL url = new URL("http://" + deviceIp + "/mode");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("POST");
                conn.setDoOutput(true);
                conn.setRequestProperty("Content-Type", "application/x-www-form-urlencoded");
                conn.setRequestProperty("Content-Length", String.valueOf(body.length()));
                conn.setConnectTimeout(3000);
                conn.setReadTimeout(3000);
                OutputStream os = conn.getOutputStream();
                os.write(body.getBytes());
                os.flush();
                os.close();
                int code = conn.getResponseCode();
                BufferedReader r = new BufferedReader(new InputStreamReader(conn.getInputStream()));
                String resp = r.readLine();
                r.close();
                conn.disconnect();
                boolean ok = code == 200 && resp != null && resp.contains("OK");
                mainHandler.post(() -> {
                    if (ok) {
                        autoMode = auto;
                        updateModeButtons();
                        log("切换到" + (auto ? "自动" : "手动") + "模式");
                        /* 确保距离轮询在运行 */
                        if (!distancePolling) {
                            startDistancePolling();
                        }
                        /* 立即触发一次距离查询刷新UI */
                        pollDistance();
                    } else {
                        Toast.makeText(this, "模式切换失败", Toast.LENGTH_SHORT).show();
                    }
                });
            } catch (Exception e) {
                mainHandler.post(() -> {
                    log("模式切换错误: " + e.getMessage());
                    Toast.makeText(this, "模式切换失败", Toast.LENGTH_SHORT).show();
                });
            }
        });
    }

    private void updateRelayDisplay(boolean on) {
        if (on) {
            relayStatus.setText("状态: 吸合 (ON)");
            relayStatus.setTextColor(getColor(R.color.success));
            relayStatus.setBackgroundResource(R.drawable.status_on_bg);
        } else {
            relayStatus.setText("状态: 断开 (OFF)");
            relayStatus.setTextColor(getColor(R.color.danger));
            relayStatus.setBackgroundResource(R.drawable.status_off_bg);
        }
    }

    private void sendCmd(String cmd, String label) {
        if (deviceIp == null) {
            Toast.makeText(this, "请先添加设备", Toast.LENGTH_SHORT).show();
            return;
        }
        if (autoMode) {
            Toast.makeText(this, "自动模式下不可手动操作", Toast.LENGTH_SHORT).show();
            return;
        }
        polling = false;
        distancePolling = false;
        mainHandler.removeCallbacksAndMessages(null);
        executor.execute(() -> {
            boolean ok = false;
            /* 自动重发3次，第1次成功就停 */
            for (int i = 0; i < 3 && !ok; i++) {
                try {
                    URL url = new URL("http://" + deviceIp + "/cmd?c=" + cmd);
                    HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                    conn.setRequestMethod("GET");
                    conn.setConnectTimeout(3000);
                    conn.setReadTimeout(3000);
                    conn.connect();
                    int code = conn.getResponseCode();
                    BufferedReader r = new BufferedReader(new InputStreamReader(conn.getInputStream()));
                    String resp = r.readLine();
                    r.close();
                    conn.disconnect();
                    if (code == 200 && resp != null && resp.contains("OK")) {
                        ok = true;
                    }
                } catch (Exception e) {
                    /* 本次失败，继续重试 */
                }
                if (!ok) {
                    try { Thread.sleep(300); } catch (InterruptedException ie) {}
                }
            }
            final boolean success = ok;
            String time = new SimpleDateFormat("HH:mm:ss", Locale.getDefault()).format(new Date());
            mainHandler.post(() -> {
                log("[" + time + "] " + label + ": " + (success ? "成功" : "失败"));
                if (success) {
                    if (cmd.equals("1")) {
                        updateRelayDisplay(true);
                    } else if (cmd.equals("0")) {
                        updateRelayDisplay(false);
                    }
                }
            });
            mainHandler.postDelayed(() -> {
                if (deviceIp != null) {
                    startPolling();
                    startDistancePolling();
                }
            }, 1500);
        });
    }

    private void log(String msg) {
        String time = new SimpleDateFormat("HH:mm:ss", Locale.getDefault()).format(new Date());
        String current = logText.getText().toString();
        logText.setText("[" + time + "] " + msg + "\n" + current);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (returningFromProvision) {
            returningFromProvision = false;
            deviceIp = null;
            getSharedPreferences(PREFS, MODE_PRIVATE)
                    .edit().remove(KEY_DEVICE_IP).apply();
            log("从配网页返回，自动搜索设备...");
            discoverDevice();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        polling = false;
        distancePolling = false;
        mainHandler.removeCallbacksAndMessages(null);
        executor.shutdown();
    }
}
