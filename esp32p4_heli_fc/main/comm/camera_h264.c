/*
 * USB Webcam (Logitech C920) — MJPEG over HTTP/TCP
 *
 * Pipeline: USB UVC → MJPEG frames → double buffer → HTTP /mjpeg (port 81)
 *
 * The webcam outputs MJPEG natively — no ISP or HW encoder needed.
 * Thermal camera (FLIR Lepton SPI) is handled separately in thermal_camera.c.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"

#include "camera_h264.h"
#include "thermal_camera.h"

#include "usb/usb_host.h"
#include "usb/uvc_host.h"

static const char *TAG = "usb_cam";

/* USB camera configuration — Logitech C920 */
#define USB_CAM_VID         0x046D  /* Logitech */
#define USB_CAM_PID         0x082D  /* C920 HD Pro Webcam */
#define USB_CAM_WIDTH       640
#define USB_CAM_HEIGHT      480
#define USB_CAM_FPS         15
#define USB_CAM_MAX_FRAME   (100 * 1024)  /* 100KB max MJPEG frame */

/* Stream target FPS over HaLow — ~43KB/frame, HaLow ~100KB/s effective.
 * 2fps x 43KB = 86KB/s fits comfortably within HaLow bandwidth. */
#define STREAM_TARGET_FPS   2
#define STREAM_INTERVAL_MS  (1000 / STREAM_TARGET_FPS)

/* Double buffer for MJPEG frames */
#define NUM_BUFS            2

/* Module state */
static struct {
    bool initialized;

    /* JPEG double buffer (MJPEG stream) */
    uint8_t *jpeg_buf[NUM_BUFS];
    size_t jpeg_size[NUM_BUFS];
    volatile int jpeg_write_idx;
    volatile int jpeg_read_idx;
    SemaphoreHandle_t frame_ready;
    SemaphoreHandle_t jpeg_mutex;

    /* Send buffer — frame is copied here under mutex before slow TCP send */
    uint8_t *send_buf;
    size_t send_buf_size;

    /* Frame statistics */
    volatile float fps;
    uint32_t frame_count;
    int64_t stats_start_time;

    /* On-demand streaming */
    volatile int mjpeg_clients;

    /* UVC handle */
    uvc_host_stream_hdl_t uvc_stream;
} s_cam = {0};


/* ========== USB Host Library Event Task ========== */

static void usb_host_lib_task(void *param)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            /* All clients deregistered */
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            /* All devices freed */
        }
    }
}


/* ========== UVC Frame Callback ========== */

static bool uvc_frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    static uint32_t s_frame_count = 0;

    if (!frame || !frame->data || frame->data_len == 0) return true;

    s_frame_count++;
    if (s_frame_count == 1) {
        const uint8_t *p = (const uint8_t *)frame->data;
        ESP_LOGI(TAG, "First frame: len=%u  bytes[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x",
                 (unsigned)frame->data_len,
                 p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
        if (p[0] == 0xFF && p[1] == 0xD8) {
            ESP_LOGI(TAG, "MJPEG format confirmed (SOI marker)");
        } else {
            ESP_LOGW(TAG, "Unexpected format — not MJPEG?");
        }
    }
    if ((s_frame_count % 150) == 0) {
        ESP_LOGI(TAG, "USB cam frame #%lu  len=%u",
                 (unsigned long)s_frame_count, (unsigned)frame->data_len);
    }

    /* Copy MJPEG frame to double buffer */
    if (xSemaphoreTake(s_cam.jpeg_mutex, 0) == pdTRUE) {
        int wr_idx = s_cam.jpeg_write_idx;
        size_t copy_len = (frame->data_len < USB_CAM_MAX_FRAME)
                          ? frame->data_len : USB_CAM_MAX_FRAME;
        memcpy(s_cam.jpeg_buf[wr_idx], frame->data, copy_len);
        s_cam.jpeg_size[wr_idx] = copy_len;
        s_cam.jpeg_write_idx = (wr_idx + 1) % NUM_BUFS;
        s_cam.jpeg_read_idx = wr_idx;
        xSemaphoreGive(s_cam.jpeg_mutex);

        /* Signal MJPEG HTTP handler */
        xSemaphoreGive(s_cam.frame_ready);
    }

    /* Update FPS stats */
    s_cam.frame_count++;
    int64_t now = esp_timer_get_time();
    int64_t elapsed = now - s_cam.stats_start_time;
    if (elapsed > 1000000) {
        s_cam.fps = (float)s_cam.frame_count * 1000000.0f / (float)elapsed;
        s_cam.frame_count = 0;
        s_cam.stats_start_time = now;
    }

    return true;
}


/* ========== UVC Stream Event Callback ========== */

static void uvc_stream_callback(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGW(TAG, "UVC transfer error");
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "USB camera disconnected");
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "Frame buffer overflow");
        break;
    default:
        break;
    }
}


/* ========== Initialization ========== */

esp_err_t camera_h264_init(void)
{
    esp_err_t ret;

    if (s_cam.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing USB webcam pipeline (MJPEG %dx%d @%dfps)",
             USB_CAM_WIDTH, USB_CAM_HEIGHT, USB_CAM_FPS);

    s_cam.frame_ready = xSemaphoreCreateCounting(NUM_BUFS, 0);
    s_cam.jpeg_mutex = xSemaphoreCreateMutex();

    /* Allocate MJPEG double buffers in PSRAM */
    for (int i = 0; i < NUM_BUFS; i++) {
        s_cam.jpeg_buf[i] = heap_caps_calloc(1, USB_CAM_MAX_FRAME, MALLOC_CAP_SPIRAM);
        if (!s_cam.jpeg_buf[i]) {
            ESP_LOGE(TAG, "Failed to allocate JPEG buffer %d", i);
            return ESP_ERR_NO_MEM;
        }
    }
    /* Send buffer: frame copied here under mutex before slow TCP send */
    s_cam.send_buf = heap_caps_calloc(1, USB_CAM_MAX_FRAME, MALLOC_CAP_SPIRAM);
    if (!s_cam.send_buf) {
        ESP_LOGE(TAG, "Failed to allocate send buffer");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "MJPEG buffers: %d x %uKB + send %uKB (PSRAM)", NUM_BUFS,
             (unsigned)(USB_CAM_MAX_FRAME / 1024),
             (unsigned)(USB_CAM_MAX_FRAME / 1024));

    /* Install USB Host Library */
    usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ret = usb_host_install(&host_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB host install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start USB Host event handling task */
    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_host", 4096, NULL, 2, NULL, 1);

    /* Install UVC driver */
    uvc_host_driver_config_t uvc_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 5,
        .xCoreID = 1,
        .create_background_task = true,
    };
    ret = uvc_host_install(&uvc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UVC host install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Open UVC stream — Logitech C920 (or any UVC webcam) */
    uvc_host_stream_config_t stream_config = {
        .event_cb = uvc_stream_callback,
        .frame_cb = uvc_frame_callback,
        .user_ctx = NULL,
        .usb = {
            .vid = USB_CAM_VID,
            .pid = USB_CAM_PID,
        },
        .vs_format = {
            .h_res = USB_CAM_WIDTH,
            .v_res = USB_CAM_HEIGHT,
            .fps = USB_CAM_FPS,
            .format = UVC_VS_FORMAT_MJPEG,
        },
        .advanced = {
            .number_of_frame_buffers = 3,
            .frame_size = USB_CAM_MAX_FRAME,
            .frame_heap_caps = MALLOC_CAP_SPIRAM,
            .number_of_urbs = 5,
            .urb_size = 20 * 1024,
        },
    };

    ESP_LOGI(TAG, "Waiting for USB camera (VID=0x%04X PID=0x%04X MJPEG %dx%d@%dfps)...",
             USB_CAM_VID, USB_CAM_PID, USB_CAM_WIDTH, USB_CAM_HEIGHT, USB_CAM_FPS);
    ret = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(15000), &s_cam.uvc_stream);
    if (ret != ESP_OK) {
        /* Exact VID/PID failed — retry accepting any UVC device */
        ESP_LOGW(TAG, "C920 open failed (%s), retrying with any UVC device...",
                 esp_err_to_name(ret));
        stream_config.usb.vid = 0;
        stream_config.usb.pid = 0;
        ret = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(15000), &s_cam.uvc_stream);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "UVC stream open failed: %s (is USB camera connected?)",
                     esp_err_to_name(ret));
            /* Return error — camera_h264_init caller will log and continue.
             * HTTP server still starts for thermal camera + status. */
            s_cam.uvc_stream = NULL;
            return ret;
        }
    }

    ESP_LOGI(TAG, "USB camera opened: %dx%d MJPEG @%dfps",
             stream_config.vs_format.h_res,
             stream_config.vs_format.v_res,
             stream_config.vs_format.fps);

    /* Start UVC streaming */
    ret = uvc_host_stream_start(s_cam.uvc_stream);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UVC stream start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_cam.initialized = true;
    s_cam.stats_start_time = esp_timer_get_time();

    ESP_LOGI(TAG, "USB webcam pipeline initialized successfully");
    return ESP_OK;
}


/* ========== HTTP Streaming Handlers ========== */

#define PART_BOUNDARY "esp32p4-halow-boundary"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req,
        "<html><body style='background:#111;color:#eee;font-family:monospace;text-align:center;padding:20px'>"
        "<h2>ESP32-P4 Helicopter</h2>"
        "<div><img id='cam' style='max-width:100%;border:1px solid #444' /></div>"
        "<p style='color:#0f0;font-size:12px'>USB Webcam MJPEG 640x480 (port 81)</p>"
        "<script>document.getElementById('cam').src='http://'+location.hostname+':81/mjpeg';</script>"
        "<p><a href='/thermal' style='color:#0af;font-size:18px'>Thermal Camera</a></p>"
        "<p><a href='/status' style='color:#0af;font-size:14px'>Status</a></p>"
        "</body></html>", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"initialized\":%s,\"resolution\":\"%dx%d\",\"fps\":%.1f,"
        "\"encoder\":\"usb_mjpeg\",\"transport\":\"http_tcp\","
        "\"mjpeg_clients\":%d,"
        "\"pipeline\":\"usb_uvc_mjpeg\","
        "\"stream_url\":\"/mjpeg\"}",
        s_cam.initialized ? "true" : "false",
        USB_CAM_WIDTH, USB_CAM_HEIGHT, s_cam.fps,
        s_cam.mjpeg_clients);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, strlen(buf));
}

/* ========== MJPEG HTTP Multipart Stream (TCP — primary video) ========== */

static esp_err_t mjpeg_stream_handler(httpd_req_t *req)
{
    if (!s_cam.initialized || !s_cam.uvc_stream) {
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "Camera not available", HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

    s_cam.mjpeg_clients++;
    ESP_LOGI(TAG, "MJPEG HTTP client connected (%d active, %dfps target)",
             s_cam.mjpeg_clients, STREAM_TARGET_FPS);

    esp_err_t res = ESP_OK;
    char part_hdr[128];
    int64_t last_send_us = 0;

    while (res == ESP_OK) {
        /* Wait for a new JPEG frame from UVC callback */
        if (xSemaphoreTake(s_cam.frame_ready, pdMS_TO_TICKS(5000)) != pdTRUE) {
            continue;  /* Timeout — keep connection alive, try again */
        }

        /* Drain extra semaphore signals (USB cam runs at 15fps,
         * we only send at STREAM_TARGET_FPS) */
        while (xSemaphoreTake(s_cam.frame_ready, 0) == pdTRUE) { }

        /* Frame rate throttle: skip if too soon since last send */
        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_send_us) < (STREAM_INTERVAL_MS * 1000)) {
            continue;
        }

        /* Copy frame to send buffer under mutex — prevents race condition
         * where UVC callback overwrites jpeg_buf during slow TCP send */
        size_t send_len = 0;
        if (xSemaphoreTake(s_cam.jpeg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            int rd_idx = s_cam.jpeg_read_idx;
            send_len = s_cam.jpeg_size[rd_idx];
            if (send_len > 0 && send_len <= USB_CAM_MAX_FRAME) {
                memcpy(s_cam.send_buf, s_cam.jpeg_buf[rd_idx], send_len);
            }
            xSemaphoreGive(s_cam.jpeg_mutex);
        }

        if (send_len == 0) continue;

        /* Send multipart boundary + JPEG content-type header */
        int hdr_len = snprintf(part_hdr, sizeof(part_hdr),
            "%sContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            STREAM_BOUNDARY, (unsigned)send_len);

        res = httpd_resp_send_chunk(req, part_hdr, hdr_len);
        if (res != ESP_OK) break;

        /* Send from send_buf — safe from UVC callback overwrites */
        res = httpd_resp_send_chunk(req, (const char *)s_cam.send_buf, send_len);
        last_send_us = esp_timer_get_time();
    }

    s_cam.mjpeg_clients--;
    ESP_LOGI(TAG, "MJPEG HTTP client disconnected (%d active)", s_cam.mjpeg_clients);
    return res;
}


/* ========== Thermal Camera HTML Viewer & Raw Endpoint ========== */

static const char THERMAL_HTML[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<title>Thermal Camera</title>"
"<style>"
"body{background:#111;color:#eee;font-family:monospace;margin:0;display:flex;"
"flex-direction:column;align-items:center;justify-content:center;height:100vh}"
"canvas{image-rendering:pixelated;border:1px solid #444}"
"#info{margin-top:8px;font-size:14px}"
"</style></head><body>"
"<canvas id='c'></canvas>"
"<div id='info'>Connecting...</div>"
"<script>"
"const canvas=document.getElementById('c');"
"const ctx=canvas.getContext('2d');"
"const info=document.getElementById('info');"
"let frames=0,lastT=performance.now(),pollMs=111,errCnt=0,tempStr='';"
"const lut=new Uint8Array(256*3);"
"for(let i=0;i<256;i++){"
"  let r,g,b;"
"  if(i<64){r=0;g=0;b=i*4;}"
"  else if(i<128){let t=(i-64)*4;r=t;g=0;b=255-t;}"
"  else if(i<192){let t=(i-128)*4;r=255;g=t;b=0;}"
"  else{let t=(i-192)*4;r=255;g=255;b=t;}"
"  lut[i*3]=r;lut[i*3+1]=g;lut[i*3+2]=b;"
"}"
"const SCALE=4;"
"let imgData=null,w=0,h=0;"
"async function poll(){"
"  try{"
"    const resp=await fetch('/thermal/raw');"
"    if(resp.status===204||!resp.ok){"
"      errCnt++;pollMs=resp.status===204?1000:Math.min(5000,111*Math.pow(2,errCnt));"
"      info.textContent=resp.status===204?'Waiting for thermal camera...':'Error '+resp.status;return;"
"    }"
"    errCnt=0;pollMs=111;"
"    const tw=parseInt(resp.headers.get('X-Thermal-Width'))||160;"
"    const th=parseInt(resp.headers.get('X-Thermal-Height'))||120;"
"    if(tw!==w||th!==h){"
"      w=tw;h=th;canvas.width=w;canvas.height=h;"
"      canvas.style.width=(w*SCALE)+'px';canvas.style.height=(h*SCALE)+'px';"
"      imgData=ctx.createImageData(w,h);"
"    }"
"    const buf=await resp.arrayBuffer();"
"    const raw=new Uint8Array(buf);"
"    const npix=w*h;"
"    let y=new Uint8Array(npix);"
"    if(raw.length===npix+4){"
"      const vmin=raw[0]|(raw[1]<<8);"
"      const vmax=raw[2]|(raw[3]<<8);"
"      for(let i=0;i<npix;i++)y[i]=raw[4+i];"
"      const cx=(w>>1),cy=(h>>1);"
"      const spotN=raw[4+cy*w+cx];"
"      const rng=vmax>vmin?vmax-vmin:1;"
"      const spotRaw=vmin+spotN*rng/255;"
"      const spotC=(spotRaw/100-273.15).toFixed(1);"
"      const tminC=(vmin/100-273.15).toFixed(1);"
"      const tmaxC=(vmax/100-273.15).toFixed(1);"
"      tempStr=' | '+tminC+'~'+tmaxC+'C  center:'+spotC+'C';"
"    }else if(raw.length>=npix){"
"      for(let i=0;i<npix;i++)y[i]=raw[i];"
"      tempStr='';"
"    }"
"    const d=imgData.data;"
"    for(let i=0;i<npix;i++){"
"      const idx=y[i];"
"      d[i*4]=lut[idx*3];d[i*4+1]=lut[idx*3+1];d[i*4+2]=lut[idx*3+2];d[i*4+3]=255;"
"    }"
"    ctx.putImageData(imgData,0,0);"
"    ctx.strokeStyle='rgba(255,255,255,0.7)';ctx.lineWidth=1;"
"    const cx2=w/2,cy2=h/2;"
"    ctx.beginPath();ctx.moveTo(cx2-4,cy2);ctx.lineTo(cx2+4,cy2);"
"    ctx.moveTo(cx2,cy2-4);ctx.lineTo(cx2,cy2+4);ctx.stroke();"
"    frames++;"
"    const now=performance.now();"
"    if(now-lastT>=1000){"
"      const fps=(frames*1000/(now-lastT)).toFixed(1);"
"      info.textContent=w+'x'+h+' Y16 | '+fps+' fps'+tempStr;"
"      frames=0;lastT=now;"
"    }"
"  }catch(e){info.textContent='Error: '+e.message;}"
"}"
"function loop(){poll().finally(()=>setTimeout(loop,pollMs));}loop();"
"</script></body></html>";

static esp_err_t thermal_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, THERMAL_HTML, strlen(THERMAL_HTML));
}

static esp_err_t thermal_raw_handler(httpd_req_t *req)
{
    if (!thermal_camera_is_active()) {
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, NULL, 0);
    }

    unsigned tw = thermal_camera_width();
    unsigned th = thermal_camera_height();
    size_t frame_sz = thermal_camera_frame_size();
    unsigned npix = tw * th;

    uint16_t *y16_buf = heap_caps_malloc(frame_sz, MALLOC_CAP_SPIRAM);
    if (!y16_buf) {
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, NULL, 0);
    }

    if (!thermal_camera_get_frame(y16_buf)) {
        free(y16_buf);
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, NULL, 0);
    }

    /* Compact format: 4-byte header (vmin_LE16, vmax_LE16) + npix bytes (8-bit normalized) */
    uint16_t vmin = 65535, vmax = 0;
    for (unsigned i = 0; i < npix; i++) {
        if (y16_buf[i] < vmin) vmin = y16_buf[i];
        if (y16_buf[i] > vmax) vmax = y16_buf[i];
    }
    uint16_t rng = (vmax > vmin) ? (vmax - vmin) : 1;

    size_t out_sz = 4 + npix;
    uint8_t *out = heap_caps_malloc(out_sz, MALLOC_CAP_SPIRAM);
    if (!out) { free(y16_buf); return httpd_resp_send(req, NULL, 0); }

    out[0] = vmin & 0xFF; out[1] = (vmin >> 8) & 0xFF;
    out[2] = vmax & 0xFF; out[3] = (vmax >> 8) & 0xFF;

    for (unsigned i = 0; i < npix; i++) {
        out[4 + i] = (uint8_t)(((uint32_t)(y16_buf[i] - vmin) * 255) / rng);
    }
    free(y16_buf);

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Expose-Headers",
                       "X-Thermal-Width,X-Thermal-Height");

    char w_str[12], h_str[12];
    snprintf(w_str, sizeof(w_str), "%u", tw);
    snprintf(h_str, sizeof(h_str), "%u", th);
    httpd_resp_set_hdr(req, "X-Thermal-Width", w_str);
    httpd_resp_set_hdr(req, "X-Thermal-Height", h_str);

    esp_err_t res = httpd_resp_send(req, (const char *)out, out_sz);
    free(out);
    return res;
}


/* ========== HTTP Server Setup ========== */

static const httpd_uri_t uri_stream = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = stream_handler,
};

static const httpd_uri_t uri_status = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
};

static const httpd_uri_t uri_thermal = {
    .uri = "/thermal",
    .method = HTTP_GET,
    .handler = thermal_page_handler,
};

static const httpd_uri_t uri_thermal_raw = {
    .uri = "/thermal/raw",
    .method = HTTP_GET,
    .handler = thermal_raw_handler,
};

static const httpd_uri_t uri_mjpeg = {
    .uri = "/mjpeg",
    .method = HTTP_GET,
    .handler = mjpeg_stream_handler,
};

httpd_handle_t camera_stream_server_start(void)
{
    /* Main HTTP server (port 80) — quick request/response handlers only */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.max_open_sockets = 4;
    config.stack_size = 8192;

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_stream);
        httpd_register_uri_handler(server, &uri_status);
        httpd_register_uri_handler(server, &uri_thermal);
        httpd_register_uri_handler(server, &uri_thermal_raw);
        ESP_LOGI(TAG, "HTTP server started (port 80)");
        ESP_LOGI(TAG, "  Thermal: http://<ip>/thermal");
        ESP_LOGI(TAG, "  Status:  http://<ip>/status");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server (port 80)");
    }

    /* MJPEG streaming server (port 81) — separate httpd task */
    httpd_config_t mjpeg_config = HTTPD_DEFAULT_CONFIG();
    mjpeg_config.server_port = 81;
    mjpeg_config.ctrl_port = 32769;
    mjpeg_config.max_uri_handlers = 2;
    mjpeg_config.max_open_sockets = 2;
    mjpeg_config.stack_size = 8192;

    httpd_handle_t mjpeg_server = NULL;
    if (httpd_start(&mjpeg_server, &mjpeg_config) == ESP_OK) {
        httpd_register_uri_handler(mjpeg_server, &uri_mjpeg);
        ESP_LOGI(TAG, "MJPEG server started (port 81)");
        ESP_LOGI(TAG, "  Video:   http://<ip>:81/mjpeg  (USB webcam MJPEG %dx%d @%dfps)",
                 USB_CAM_WIDTH, USB_CAM_HEIGHT, USB_CAM_FPS);
    } else {
        ESP_LOGE(TAG, "Failed to start MJPEG server (port 81)");
    }

    return server;
}

float camera_get_fps(void)
{
    return s_cam.fps;
}
