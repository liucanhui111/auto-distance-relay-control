package com.liucanhui.relaycontrol;

import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.provider.Settings;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import org.json.JSONArray;
import org.json.JSONObject;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import okhttp3.MediaType;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;

/**
 * 配网引导界面
 *
 * 流程:
 * 1. 提示用户连接 ESP32_Setup 热点
 * 2. App 通过 HTTP GET 192.168.4.1/scan 从 ESP32 获取周边 WiFi 列表
 * 3. 用户选择 WiFi + 输入密码 → POST 到 192.168.4.1/config
 * 4. 成功后返回控制界面
 *
 * 不需要手机定位权限，WiFi 扫描由 ESP32 完成
 */
public class ProvisionActivity extends AppCompatActivity {

    private static final String ESP32_URL = "http://192.168.4.1";

    private Button btnOpenWifi, btnRefresh, btnSend, btnBackToControl;
    private LinearLayout wifiListContainer;
    private TextView tvSelectedSsid, tvStatus, tvLog, tvScanHint;
    private EditText etPassword;

    private String selectedSsid = null;

    private OkHttpClient httpClient;
    private Handler mainHandler;
    private LayoutInflater inflater;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        try {
            setContentView(R.layout.activity_provision);
            initViews();
        } catch (Exception e) {
            android.util.Log.e("ProvisionActivity", "onCreate crash", e);
            Toast.makeText(this, "界面初始化失败: " + e.getMessage(), Toast.LENGTH_LONG).show();
            finish();
            return;
        }
    }

    private void initViews() {
        inflater = LayoutInflater.from(this);
        httpClient = new OkHttpClient.Builder()
                .connectTimeout(5, TimeUnit.SECONDS)
                .readTimeout(10, TimeUnit.SECONDS)
                .writeTimeout(5, TimeUnit.SECONDS)
                .build();
        mainHandler = new Handler(Looper.getMainLooper());

        btnOpenWifi = findViewById(R.id.btnOpenWifiSettings);
        btnRefresh = findViewById(R.id.btnRefreshWifi);
        btnSend = findViewById(R.id.btnSendConfig);
        btnBackToControl = findViewById(R.id.btnBackToControl);
        wifiListContainer = findViewById(R.id.wifiListContainer);
        tvSelectedSsid = findViewById(R.id.tvSelectedSsid);
        tvStatus = findViewById(R.id.tvProvisionStatus);
        tvLog = findViewById(R.id.tvLog);
        tvScanHint = findViewById(R.id.tvScanHint);
        etPassword = findViewById(R.id.etPassword);

        btnOpenWifi.setOnClickListener(v -> {
            try {
                startActivity(new Intent(Settings.ACTION_WIFI_SETTINGS));
                log("已打开WiFi设置，请连接 ESP32_Setup");
            } catch (Exception e) {
                Toast.makeText(this, "无法打开WiFi设置", Toast.LENGTH_SHORT).show();
            }
        });

        btnRefresh.setOnClickListener(v -> scanWifiFromEsp32());

        btnSend.setOnClickListener(v -> {
            String pass = etPassword.getText().toString().trim();
            if (selectedSsid == null || selectedSsid.isEmpty()) {
                Toast.makeText(this, "请先选择WiFi", Toast.LENGTH_SHORT).show();
                return;
            }
            sendConfig(selectedSsid, pass);
        });

        btnBackToControl.setOnClickListener(v -> finish());
    }

    /* ========== 从 ESP32 获取 WiFi 扫描结果 ========== */

    private void scanWifiFromEsp32() {
        btnRefresh.setEnabled(false);
        tvScanHint.setText("正在从ESP32获取WiFi列表...");
        log("请求 ESP32 扫描周边 WiFi...");

        new Thread(() -> {
            try {
                Request req = new Request.Builder()
                        .url(ESP32_URL + "/scan")
                        .get()
                        .build();
                Response resp = httpClient.newCall(req).execute();
                boolean ok = resp.isSuccessful();
                String body = resp.body() != null ? resp.body().string() : "";
                resp.close();

                if (!ok && body.isEmpty()) {
                    mainHandler.post(() -> onScanFailed("无法连接ESP32，请先连接 ESP32_Setup 热点"));
                    return;
                }

                List<WifiItem> items = new ArrayList<>();
                JSONArray arr = new JSONArray(body);
                for (int i = 0; i < arr.length(); i++) {
                    JSONObject o = arr.getJSONObject(i);
                    items.add(new WifiItem(
                            o.optString("ssid", ""),
                            o.optInt("rssi", -100),
                            o.optInt("secure", 0) == 1));
                }

                mainHandler.post(() -> onScanSuccess(items));
            } catch (Exception e) {
                String msg = e.getMessage();
                if (msg == null) msg = "未知错误";
                final String errMsg = msg;
                mainHandler.post(() -> onScanFailed("扫描失败: " + errMsg + "\n请确认已连接 ESP32_Setup"));
            }
        }).start();
    }

    private void onScanSuccess(List<WifiItem> items) {
        btnRefresh.setEnabled(true);
        wifiListContainer.removeAllViews();

        if (items.isEmpty()) {
            tvScanHint.setText("未扫描到WiFi，请确认ESP32已启动配网模式");
            log("ESP32 返回空列表");
            return;
        }

        tvScanHint.setText("发现 " + items.size() + " 个WiFi，请选择");
        log("扫描到 " + items.size() + " 个WiFi");

        for (WifiItem item : items) {
            View row = inflater.inflate(R.layout.item_wifi, wifiListContainer, false);
            TextView tvSsid = row.findViewById(R.id.tvWifiSsid);
            TextView tvMeta = row.findViewById(R.id.tvWifiMeta);
            TextView tvLevel = row.findViewById(R.id.tvWifiLevel);

            tvSsid.setText(item.ssid);
            tvMeta.setText((item.secure ? "加密" : "开放") + " · " + item.rssi + "dBm");
            tvLevel.setText(rssiLabel(item.rssi));

            row.setOnClickListener(v -> {
                selectedSsid = item.ssid;
                tvSelectedSsid.setText(selectedSsid);
                Toast.makeText(this, "已选择: " + selectedSsid, Toast.LENGTH_SHORT).show();
                etPassword.requestFocus();
                log("已选择 WiFi: " + selectedSsid);
            });

            wifiListContainer.addView(row);
        }
    }

    private void onScanFailed(String msg) {
        btnRefresh.setEnabled(true);
        tvScanHint.setText("扫描失败");
        log(msg);
        Toast.makeText(this, msg, Toast.LENGTH_LONG).show();
    }

    private static String rssiLabel(int dBm) {
        if (dBm >= -50) return "强";
        if (dBm >= -65) return "较强";
        if (dBm >= -75) return "中";
        return "弱";
    }

    /* ========== 发送配置 ========== */

    private void sendConfig(String ssid, String password) {
        btnSend.setEnabled(false);
        tvStatus.setText("正在检查设备连通性...");
        log("发送配置: ssid=" + ssid);

        new Thread(() -> {
            // 先 GET /status 确认连通
            if (!checkDeviceReachable()) {
                mainHandler.post(() -> {
                    log("无法连接 192.168.4.1，请先连接 ESP32_Setup 热点");
                    tvStatus.setText("无法连接设备\n请确认已连接 ESP32_Setup 热点");
                    Toast.makeText(this, "请先连接ESP32_Setup热点", Toast.LENGTH_LONG).show();
                    btnSend.setEnabled(true);
                });
                return;
            }

            mainHandler.post(() -> tvStatus.setText("设备已连通，正在发送WiFi配置..."));
            try {
                JSONObject json = new JSONObject();
                json.put("ssid", ssid);
                json.put("password", password);

                RequestBody body = RequestBody.create(
                        MediaType.get("application/json; charset=utf-8"),
                        json.toString());

                Request request = new Request.Builder()
                        .url(ESP32_URL + "/config")
                        .post(body)
                        .build();

                Response response = httpClient.newCall(request).execute();
                String respStr = response.body() != null ? response.body().string() : "";
                boolean ok = response.isSuccessful() && respStr.contains("OK");
                response.close();

                if (ok) {
                    mainHandler.post(() -> {
                        log("配网成功！");
                        tvStatus.setText("配网成功！ESP32正在连接家庭WiFi\n请切回家庭WiFi后点击下方按钮");
                        btnBackToControl.setVisibility(View.VISIBLE);
                        btnSend.setVisibility(View.GONE);
                        Toast.makeText(this, "配网成功", Toast.LENGTH_LONG).show();
                    });
                } else {
                    final String fail = respStr;
                    mainHandler.post(() -> {
                        log("配网失败: " + fail);
                        tvStatus.setText("配网失败: " + fail + "\n请重试");
                        btnSend.setEnabled(true);
                    });
                }
            } catch (Exception e) {
                String errMsg = e.getMessage();
                if (errMsg == null) errMsg = "未知错误";
                final String finalErrMsg = errMsg;
                mainHandler.post(() -> {
                    log("错误: " + finalErrMsg);
                    tvStatus.setText("发送失败: " + finalErrMsg + "\n请确认已连接 ESP32_Setup");
                    Toast.makeText(this, "发送失败，请重试", Toast.LENGTH_LONG).show();
                    btnSend.setEnabled(true);
                });
            }
        }).start();
    }

    private boolean checkDeviceReachable() {
        try {
            Request req = new Request.Builder()
                    .url(ESP32_URL + "/status")
                    .get()
                    .build();
            Response resp = httpClient.newCall(req).execute();
            boolean ok = resp.isSuccessful();
            resp.close();
            return ok;
        } catch (Exception e) {
            return false;
        }
    }

    /* ========== 工具类 ========== */

    private static class WifiItem {
        final String ssid;
        final int rssi;
        final boolean secure;

        WifiItem(String ssid, int rssi, boolean secure) {
            this.ssid = ssid;
            this.rssi = rssi;
            this.secure = secure;
        }
    }

    private void log(String msg) {
        String time = new SimpleDateFormat("HH:mm:ss", Locale.getDefault()).format(new Date());
        String current = tvLog.getText().toString();
        tvLog.setText("[" + time + "] " + msg + "\n" + current);
    }
}
