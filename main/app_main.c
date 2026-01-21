#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <math.h> 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_sntp.h"
#include "mdns.h"

#include "cJSON.h"

#include "u8g2.h"
#include "u8g2_esp32_hal.h"

// ---------------- OLED Pins (ESP32-C3 Mini) ----------------
#define I2C_SDA_PIN     2
#define I2C_SCL_PIN     3
#define I2C_ADDRESS     0x78   // 0x3C<<1

static u8g2_t u8g2;
// Mutex to protect the OLED from being accessed by multiple tasks at once
static SemaphoreHandle_t xGuiSemaphore = NULL;

// ---------------- Wi-Fi / NVS ----------------
#define WIFI_NAMESPACE          "bsqcfg"
#define KEY_WIFI_SSID           "ssid"
#define KEY_WIFI_PASS           "pass"
#define KEY_MENU_JSON           "menu"
#define KEY_LAST_EPOCH          "last_epoch"

#define SOFTAP_SSID             "BSQ_TIMER"
#define SOFTAP_PASS             "12345678"

static const char *TAG = "BSQ_TIMER";
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// ---------------- App State ----------------
typedef enum {
    SCREEN_CLOCK = 0,
    SCREEN_MENU  = 1,
    SCREEN_TIMER = 2,
    SCREEN_BELL  = 3,
} screen_mode_t;

// Global state protected by atomic updates or semaphore where needed
static struct {
    volatile screen_mode_t screen;
    volatile int timer_running;
    volatile int timer_total;
    volatile int timer_remaining;
    char selected_path[128];     // current menu path
    volatile int64_t last_interaction_ms;
    bool sta_connected;
    bool state_changed;          // signal for UI to refresh immediately
} g_state = {
    .screen = SCREEN_CLOCK,
    .timer_running = 0,
    .timer_total = 0,
    .timer_remaining = 0,
    .selected_path = "/",
    .last_interaction_ms = 0,
    .sta_connected = false,
    .state_changed = false
};

// Menu stored as JSON string in NVS
static char *g_menu_json = NULL;

// ---------------- Helper Functions ----------------

static int64_t now_ms(void) {
    return (int64_t)(esp_timer_get_time() / 1000);
}

// Signal the UI task that something changed and it should wake up/redraw
static void trigger_ui_update(void) {
    g_state.state_changed = true;
    g_state.last_interaction_ms = now_ms();
}

// Wake up from clock screen if needed
static void wake_screen(void) {
    if (g_state.screen == SCREEN_CLOCK) {
        g_state.screen = SCREEN_MENU;
    }
    trigger_ui_update();
}

// ---------------- NVS Helpers ----------------

static esp_err_t nvs_read_str(const char *key, char **out) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    size_t len = 0;
    err = nvs_get_str(h, key, NULL, &len);
    if (err != ESP_OK) { nvs_close(h); return err; }

    char *buf = (char*)calloc(1, len);
    if (!buf) { nvs_close(h); return ESP_ERR_NO_MEM; }

    err = nvs_get_str(h, key, buf, &len);
    nvs_close(h);
    if (err != ESP_OK) { free(buf); return err; }

    *out = buf;
    return ESP_OK;
}

static esp_err_t nvs_write_str(const char *key, const char *val) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_str(h, key, val);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

// ---------------- Wi-Fi Credential Helpers ----------------

static void wifi_save_credentials(const char *ssid, const char *pass) {
    if (ssid) nvs_write_str(KEY_WIFI_SSID, ssid);
    if (pass) nvs_write_str(KEY_WIFI_PASS, pass);
}

static void wifi_load_credentials(char **ssid, char **pass) {
    *ssid = NULL; *pass = NULL;
    nvs_read_str(KEY_WIFI_SSID, ssid);
    nvs_read_str(KEY_WIFI_PASS, pass);
}

// ---------------- Menu Logic ----------------

static const char *DEFAULT_MENU =
"{"
"  \"name\":\"root\","
"  \"type\":\"folder\","
"  \"children\":["
"    {\"name\":\"Sample Folder\",\"type\":\"folder\",\"children\":["
"        {\"name\":\"Fixed 150s\",\"type\":\"timer\",\"mode\":\"fixed\",\"fixed\":150},"
"        {\"name\":\"Variable Timer\",\"type\":\"timer\",\"mode\":\"variable\"}"
"    ]}"
"  ]"
"}";

static void load_menu_from_nvs(void) {
    char *s = NULL;
    if (nvs_read_str(KEY_MENU_JSON, &s) == ESP_OK && s && strlen(s) > 5) {
        g_menu_json = s;
        ESP_LOGI(TAG, "Loaded menu from NVS");
        return;
    }
    g_menu_json = strdup(DEFAULT_MENU);
    nvs_write_str(KEY_MENU_JSON, g_menu_json);
}

static void save_menu_to_nvs(const char *json) {
    if (g_menu_json) free(g_menu_json);
    g_menu_json = strdup(json);
    nvs_write_str(KEY_MENU_JSON, g_menu_json);
}

static cJSON* menu_parse(void) {
    return cJSON_Parse(g_menu_json ? g_menu_json : DEFAULT_MENU);
}

static char* menu_stringify(cJSON *root) {
    return cJSON_PrintUnformatted(root);
}

static cJSON* menu_find_node(cJSON *root, const char *path) {
    if (!root || !path) return NULL;
    if (strcmp(path, "/") == 0) return root;

    char tmp[256];
    strncpy(tmp, path, sizeof(tmp)-1);
    tmp[sizeof(tmp)-1] = 0;

    cJSON *node = root;
    char *saveptr = NULL;
    char *p = strtok_r(tmp, "/", &saveptr);
    while (p) {
        cJSON *children = cJSON_GetObjectItem(node, "children");
        if (!cJSON_IsArray(children)) return NULL;

        cJSON *found = NULL;
        cJSON *ch = NULL;
        cJSON_ArrayForEach(ch, children) {
            cJSON *nm = cJSON_GetObjectItem(ch, "name");
            if (cJSON_IsString(nm) && strcmp(nm->valuestring, p) == 0) {
                found = ch;
                break;
            }
        }
        if (!found) return NULL;
        node = found;
        p = strtok_r(NULL, "/", &saveptr);
    }
    return node;
}

static esp_err_t menu_add_item(const char *parent_path, const char *name, const char *type, const char *mode, int fixed) {
    if (!parent_path || !name) return ESP_ERR_INVALID_ARG;
    cJSON *root = menu_parse();
    if (!root) return ESP_FAIL;

    cJSON *parent = menu_find_node(root, parent_path);
    if (!parent) { cJSON_Delete(root); return ESP_ERR_NOT_FOUND; }

    cJSON *children = cJSON_GetObjectItem(parent, "children");
    if (!cJSON_IsArray(children)) {
        children = cJSON_AddArrayToObject(parent, "children");
    }
    
    // Check duplicates
    cJSON *ch;
    cJSON_ArrayForEach(ch, children) {
        cJSON *nm = cJSON_GetObjectItem(ch, "name");
        if(cJSON_IsString(nm) && strcmp(nm->valuestring, name)==0) {
            cJSON_Delete(root); return ESP_ERR_INVALID_STATE;
        }
    }

    cJSON *item = cJSON_CreateObject();
    cJSON_AddStringToObject(item, "name", name);

    if (strcmp(type, "folder") == 0) {
        cJSON_AddStringToObject(item, "type", "folder");
        cJSON_AddItemToObject(item, "children", cJSON_CreateArray());
    } else {
        cJSON_AddStringToObject(item, "type", "timer");
        if (mode && strcmp(mode, "fixed") == 0) {
            cJSON_AddStringToObject(item, "mode", "fixed");
            cJSON_AddNumberToObject(item, "fixed", fixed > 0 ? fixed : 150);
        } else {
            cJSON_AddStringToObject(item, "mode", "variable");
        }
    }

    cJSON_AddItemToArray(children, item);
    char *out = menu_stringify(root);
    save_menu_to_nvs(out);
    free(out);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t menu_delete_item(const char *parent_path, const char *name) {
    cJSON *root = menu_parse();
    if (!root) return ESP_FAIL;

    cJSON *parent = menu_find_node(root, parent_path);
    if (!parent) { cJSON_Delete(root); return ESP_ERR_NOT_FOUND; }

    cJSON *children = cJSON_GetObjectItem(parent, "children");
    if (!cJSON_IsArray(children)) { cJSON_Delete(root); return ESP_ERR_NOT_FOUND; }

    int idx = 0;
    cJSON *ch = NULL;
    int found_idx = -1;
    cJSON_ArrayForEach(ch, children) {
        cJSON *nm = cJSON_GetObjectItem(ch, "name");
        if (cJSON_IsString(nm) && strcmp(nm->valuestring, name) == 0) {
            found_idx = idx;
            break;
        }
        idx++;
    }

    if (found_idx >= 0) {
        cJSON_DeleteItemFromArray(children, found_idx);
        char *out = menu_stringify(root);
        save_menu_to_nvs(out);
        free(out);
        cJSON_Delete(root);
        return ESP_OK;
    }

    cJSON_Delete(root);
    return ESP_ERR_NOT_FOUND;
}

// ---------------- OLED UI Functions ----------------

static void oled_init(void) {
    xGuiSemaphore = xSemaphoreCreateMutex();
    u8g2_esp32_hal_t hal = U8G2_ESP32_HAL_DEFAULT;
    hal.bus.i2c.sda = I2C_SDA_PIN;
    hal.bus.i2c.scl = I2C_SCL_PIN;
    u8g2_esp32_hal_init(hal);

    u8g2_Setup_sh1106_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb
    );
    u8x8_SetI2CAddress(&u8g2.u8x8, I2C_ADDRESS);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
}
static void draw_clock_screen(void) {
    time_t now; 
    time(&now); 
    struct tm t; 
    localtime_r(&now, &t);

    // CHANGED: Increased font size to ncenB14 (same as seconds)
    // Adjusted Y from 20 to 18 to keep it near top
    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&u8g2, 0, 18, "BSQ Timer");

    if (t.tm_year < (2020 - 1900)) {
        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
        u8g2_DrawStr(&u8g2, 8, 42, "No Sync");
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
        u8g2_DrawStr(&u8g2, 8, 60, g_state.sta_connected?"WiFi OK":"No WiFi");
    } else {
        char b1[32], b2[32];
        snprintf(b1, sizeof(b1), "%02d:%02d", t.tm_hour, t.tm_min);
        snprintf(b2, sizeof(b2), ":%02d", t.tm_sec);
        
        // CHANGED: Moved main time down slightly (Y=60 -> Y=64) to maximize space
        u8g2_SetFont(&u8g2, u8g2_font_logisoso32_tf);
        u8g2_DrawStr(&u8g2, 0, 64, b1);
        
        // CHANGED: Seconds are fine, just ensured they align with the bottom
        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
        u8g2_DrawStr(&u8g2, 95, 62, b2);
    }
}
//static void draw_clock_screen(void) {
//    time_t now; 
//    time(&now); 
//    struct tm t; 
//    localtime_r(&now, &t);
//
//    u8g2_SetFont(&u8g2, u8g2_font_6x12_tr);
//    u8g2_DrawStr(&u8g2, 0, 20, "BSQ Timer");
//
//    if (t.tm_year < (2020 - 1900)) {
//        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
//        u8g2_DrawStr(&u8g2, 8, 42, "No Sync");
//        u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
//        u8g2_DrawStr(&u8g2, 8, 60, g_state.sta_connected?"WiFi OK":"No WiFi");
//    } else {
//        char b1[32], b2[32];
//        snprintf(b1, sizeof(b1), "%02d:%02d", t.tm_hour, t.tm_min);
//        snprintf(b2, sizeof(b2), ":%02d", t.tm_sec);
//        u8g2_SetFont(&u8g2, u8g2_font_logisoso32_tf);
//        u8g2_DrawStr(&u8g2, 0, 60, b1);
//        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
//        u8g2_DrawStr(&u8g2, 95, 58, b2);
//    }
//} this is for Small font 

static void draw_menu_screen(void) {
    // 1. Draw Header
    const char *path = g_state.selected_path;
    const char *header = "Main Menu";
    if (strcmp(path, "/") != 0) {
        const char *p = strrchr(path, '/');
        if (p) header = p + 1; else header = path;
    }
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tr);
    u8g2_DrawStr(&u8g2, 0, 10, header);
    u8g2_DrawHLine(&u8g2, 0, 13, 128);

    // 2. Parse Menu to find current node
    cJSON *root = menu_parse();
    cJSON *node = menu_find_node(root, path);
    if (!node) node = root; // safe fallback

    // 3. CHECK: Is this a timer? (FIX for "Empty" display)
    cJSON *type = cJSON_GetObjectItem(node, "type");
    bool isTimer = (type && cJSON_IsString(type) && strcmp(type->valuestring, "timer") == 0);

    if (isTimer) {
        // --- Show Timer Details if it is a timer node ---
        u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
        u8g2_DrawStr(&u8g2, 0, 30, "Timer Ready:");
        
        cJSON *mode = cJSON_GetObjectItem(node, "mode");
        cJSON *fixed = cJSON_GetObjectItem(node, "fixed");
        char buf[32];
        if (mode && strcmp(mode->valuestring,"fixed")==0 && fixed) {
            snprintf(buf, sizeof(buf), "%ds", fixed->valueint);
        } else {
            snprintf(buf, sizeof(buf), "Variable");
        }
        
        // Center the time/mode text
        u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tr);
        int w = u8g2_GetStrWidth(&u8g2, buf);
        u8g2_DrawStr(&u8g2, 64 - (w/2), 50, buf);

        u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
        u8g2_DrawStr(&u8g2, 28, 62, "[Start on Web]");
    } else {
        // --- Show Folder List ---
        cJSON *children = cJSON_GetObjectItem(node, "children");
        int y = 26;
        if (cJSON_IsArray(children)) {
            int count = 0;
            cJSON *ch;
            cJSON_ArrayForEach(ch, children) {
                if (count >= 3) { u8g2_DrawStr(&u8g2, 50, y, "..."); break; }
                cJSON *nm = cJSON_GetObjectItem(ch, "name");
                cJSON *tp = cJSON_GetObjectItem(ch, "type");
                if (cJSON_IsString(nm)) {
                    char buf[32];
                    bool isF = (strcmp(tp->valuestring, "folder") == 0);
                    snprintf(buf, sizeof(buf), "%s %s", isF ? ">" : "*", nm->valuestring);
                    u8g2_DrawStr(&u8g2, 4, y, buf);
                    y += 12; count++;
                }
            }
            if (count==0) u8g2_DrawStr(&u8g2, 4, y, "(Empty)");
        } else {
            u8g2_DrawStr(&u8g2, 4, y, "(Empty)");
        }
    }
    cJSON_Delete(root);
}

static void draw_timer_screen(void) {
    // 1. Calculate Progress
    float progress = 0.0f;
    if (g_state.timer_total > 0) {
        progress = (float)g_state.timer_remaining / (float)g_state.timer_total;
    }
    
    // Clamp progress
    if (progress > 1.0f) progress = 1.0f;
    if (progress < 0.0f) progress = 0.0f;

    // 2. Draw Arc (Double Thickness)
    // 360 degrees * progress. Start at -90 degrees (12 o'clock)
    int max_angle = (int)(360.0f * progress);
    
    // Draw arc point by point to create a progressive ring
    // i goes from 0 to max_angle.
    for (int i = 0; i <= max_angle; i++) {
        // Convert degree offset to radians. -90 degrees puts 0 at the top.
        float rad = (i - 90) * 0.0174532925f; // PI / 180
        
        // Outer ring (r=28)
        int x1 = 64 + (int)(28 * cosf(rad));
        int y1 = 32 + (int)(28 * sinf(rad));
        u8g2_DrawPixel(&u8g2, x1, y1);
        
        // Inner ring (r=27) - Double thickness
        int x2 = 64 + (int)(27 * cosf(rad));
        int y2 = 32 + (int)(27 * sinf(rad));
        u8g2_DrawPixel(&u8g2, x2, y2);
    }

    // 3. Draw Center Time
    int sec = g_state.timer_remaining;
    int m = sec / 60;
    int s = sec % 60;
    char buf[16];
    snprintf(buf, sizeof(buf), "%d:%02d", m, s);

    u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tr);
    int w = u8g2_GetStrWidth(&u8g2, buf);
    // Center is (64, 32). Font height ~12. Baseline ~37 centers it visually.
    u8g2_DrawStr(&u8g2, 64 - (w/2), 37, buf);
}

static void draw_bell_screen(void) {
    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_4x_t);
    u8g2_DrawGlyph(&u8g2, 48, 48, 0x0078); 
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 38, 62, "TIME UP!");
}

// Wrapper to draw safely with Mutex
static void oled_draw_wrapper(screen_mode_t sc) {
    if (xGuiSemaphore && xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
        u8g2_ClearBuffer(&u8g2);
        switch (sc) {
            case SCREEN_CLOCK: draw_clock_screen(); break;
            case SCREEN_MENU:  draw_menu_screen(); break;
            case SCREEN_TIMER: draw_timer_screen(); break;
            case SCREEN_BELL:  draw_bell_screen(); break;
        }
        u8g2_SendBuffer(&u8g2);
        xSemaphoreGive(xGuiSemaphore);
    }
}

// Smooth Fade Transition (Contrast based)
static void oled_transition_fade(screen_mode_t to_screen) {
    // Fade OUT
    for (int c = 255; c >= 0; c -= 25) {
        if (xGuiSemaphore && xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            u8g2_SetContrast(&u8g2, c > 0 ? c : 0);
            xSemaphoreGive(xGuiSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    // Switch & Draw Hidden
    oled_draw_wrapper(to_screen);
    
    // Fade IN
    for (int c = 0; c <= 255; c += 25) {
        if (xGuiSemaphore && xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            u8g2_SetContrast(&u8g2, c < 255 ? c : 255);
            xSemaphoreGive(xGuiSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ---------------- UI Task (The Loop) ----------------

static void ui_task(void *arg) {
    int64_t last_tick = now_ms();
    screen_mode_t current_visual_screen = SCREEN_CLOCK;
    oled_draw_wrapper(SCREEN_CLOCK);

    while (1) {
        int64_t t = now_ms();
        bool refresh_needed = false;

        // 1. Handle Timer Countdown
        if (t - last_tick >= 1000) {
            last_tick = t;
            if (g_state.timer_running && g_state.timer_remaining > 0) {
                g_state.timer_remaining--;
                
                // Force switch to TIMER screen if running and not already there
                if (g_state.screen != SCREEN_TIMER && g_state.screen != SCREEN_BELL) {
                    g_state.screen = SCREEN_TIMER;
                }
                
                if (g_state.timer_remaining == 0) {
                    g_state.timer_running = 0;
                    g_state.screen = SCREEN_BELL;
                } else {
                    refresh_needed = true;
                }
            } else if (g_state.screen == SCREEN_CLOCK) {
                // Clock updates every second
                refresh_needed = true;
            }
        }

        // 2. Bell Duration Logic
        static int64_t bell_start = 0;
        if (g_state.screen == SCREEN_BELL) {
            if (bell_start == 0) bell_start = t;
            if (t - bell_start >= 4000) { // 4s bell
                bell_start = 0;
                g_state.screen = SCREEN_MENU;
                trigger_ui_update();
            }
        } else {
            bell_start = 0;
        }

        // 3. Screen Transition
        if (g_state.screen != current_visual_screen) {
            oled_transition_fade(g_state.screen);
            current_visual_screen = g_state.screen;
            refresh_needed = false; // Already drew
            g_state.state_changed = false;
        } else if (g_state.state_changed) {
            // Immediate redraw requested by HTTP
            oled_draw_wrapper(current_visual_screen);
            g_state.state_changed = false;
            refresh_needed = false;
        }

        // 4. Regular Refresh (Timer tick, Clock tick)
        if (refresh_needed && current_visual_screen == g_state.screen) {
            oled_draw_wrapper(current_visual_screen);
        }

        // 5. Inactivity Timeout (30s)
        if (!g_state.timer_running && g_state.screen != SCREEN_CLOCK && g_state.screen != SCREEN_BELL) {
            if ((t - g_state.last_interaction_ms) > 30000) {
                g_state.screen = SCREEN_CLOCK; // UI loop will catch this next iteration
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------------- HTTP Strings & Handlers ----------------

// ADMIN HTML
static const char *HTML_ADMIN =
"<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>BSQ Admin</title>"
"<style>"
"body{font-family:system-ui;margin:0;background:#0b1220;color:#e8eefc}"
".top{position:sticky;top:0;background:#101a33;padding:14px 16px;font-weight:700;border-bottom:1px solid #1f2b52;z-index:10}"
".wrap{padding:16px;max-width:900px;margin:auto}"
".card{background:#0f1a33;border:1px solid #1f2b52;border-radius:14px;padding:14px;margin:12px 0}"
"input,select,button{width:100%;padding:12px;border-radius:12px;border:1px solid #2a3a72;background:#0b1220;color:#e8eefc;box-sizing:border-box}"
"button{cursor:pointer;background:#2a66ff;border:none;font-weight:700;margin-top:5px}"
"button.del{background:#ff2a2a;width:auto;padding:6px 12px;margin:0;font-size:12px}"
".row{display:grid;grid-template-columns:1fr 1fr;gap:10px}"
".small{opacity:.8;font-size:13px;margin-top:5px}"
"pre{white-space:pre-wrap;word-break:break-word;background:#0b1220;padding:12px;border-radius:12px;border:1px solid #1f2b52;font-size:11px}"
".item-row{display:flex;justify-content:space-between;align-items:center;padding:8px;border-bottom:1px solid #1f2b52}"
"</style></head><body>"
"<div class='top'>BSQ Admin Panel</div>"
"<div class='wrap'>"
"<div class='card'><h3>Wi-Fi Setup</h3><div class='row'><input id='ssid' placeholder='WiFi SSID'><input id='pass' placeholder='WiFi Password' type='password'></div><button onclick='saveWifi()'>Save Wi-Fi</button><div class='small' id='wifistatus'></div></div>"
"<div class='card'><h3>Add Item</h3><input id='parent' placeholder='Parent path (e.g. /)' value='/' /><input id='name' placeholder='Name' style='margin-top:10px'/><div class='row' style='margin-top:10px'><select id='type' onchange='typeChanged()'><option value='folder'>Sub Folder</option><option value='timer'>Timer</option></select><select id='mode' style='display:none' onchange='modeChanged()'><option value='fixed'>Fixed Time</option><option value='variable'>Input Time</option></select></div><input id='fixed' placeholder='Seconds' style='display:none;margin-top:10px'/><button onclick='addItem()'>Add Item</button><div class='small' id='addstatus'></div></div>"
"<div class='card'><h3>Manage Menu</h3><div id='menu-list'></div><button onclick='refreshMenu()'>Refresh List</button></div>"
"<div class='card'><h3>Raw JSON</h3><pre id='menu'></pre></div>"
"</div><script>"
"function typeChanged(){const t=document.getElementById('type').value;const mode=document.getElementById('mode');const fixed=document.getElementById('fixed');if(t==='timer'){mode.style.display='block';modeChanged();}else{mode.style.display='none';fixed.style.display='none';}}"
"function modeChanged(){const m=document.getElementById('mode').value;document.getElementById('fixed').style.display=(m==='fixed')?'block':'none';}"
"async function api(path,opts){const r=await fetch(path,opts);const txt=await r.text();try{return JSON.parse(txt);}catch(e){return {raw:txt,ok:r.ok};}}"
"function renderRecursive(node,path,container){if(!node.children)return;node.children.forEach(ch=>{const div=document.createElement('div');div.className='item-row';const currentPath=path==='/'?'/'+ch.name:path+'/'+ch.name;div.innerHTML=`<div>${ch.type==='folder'?'📁':'⏱️'} <b>${ch.name}</b> <span style='opacity:0.5;font-size:0.8em'>${currentPath}</span></div>`;const btn=document.createElement('button');btn.className='del';btn.textContent='Delete';btn.onclick=()=>deleteItem(path,ch.name);div.appendChild(btn);container.appendChild(div);if(ch.type==='folder')renderRecursive(ch,currentPath,container);});}"
"async function refreshMenu(){const j=await api('/api/menu');document.getElementById('menu').textContent=JSON.stringify(j,null,2);const list=document.getElementById('menu-list');list.innerHTML='';renderRecursive(j,'/',list);}"
"async function addItem(){const parent=document.getElementById('parent').value.trim();const name=document.getElementById('name').value.trim();const type=document.getElementById('type').value;const mode=document.getElementById('mode').value;const fixed=parseInt(document.getElementById('fixed').value||'0',10);const body={parent,name,type,mode,fixed};const j=await api('/api/admin/add',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});document.getElementById('addstatus').textContent=j.ok?'Added!':'Error: '+(j.error||JSON.stringify(j));refreshMenu();}"
"async function deleteItem(parentPath,name){if(!confirm('Delete '+name+'?'))return;const body={parent:parentPath,name:name};const j=await api('/api/admin/delete',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});if(!j.ok)alert('Error: '+j.error);refreshMenu();}"
"async function saveWifi(){const ssid=document.getElementById('ssid').value.trim();const pass=document.getElementById('pass').value;const j=await api('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid,pass})});document.getElementById('wifistatus').textContent=j.ok?'Saved. Rebooting...':'Error';}"
"refreshMenu();typeChanged();</script></body></html>";

// USER HTML
static const char *HTML_USER =
"<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>BSQ Timer</title>"
"<style>"
":root { --bg: #0b1220; --card: #141e33; --border: #233050; --primary: #3b82f6; --text: #e8eefc; }"
"body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif; margin: 0; background: var(--bg); color: var(--text); }"
".top { background: var(--card); padding: 16px; border-bottom: 1px solid var(--border); display: flex; justify-content: space-between; align-items: center; position: sticky; top: 0; z-index: 100; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1); }"
".logo { font-weight: 900; font-size: 1.2rem; letter-spacing: -0.5px; }"
".status { font-size: 0.75rem; padding: 4px 8px; border-radius: 99px; background: rgba(255,255,255,0.1); }"
".container { max-width: 600px; margin: 0 auto; padding: 16px; }"
".section-title { text-transform: uppercase; font-size: 0.75rem; letter-spacing: 1px; color: #64748b; margin: 24px 0 8px 4px; font-weight: 700; }"
".card { background: var(--card); border: 1px solid var(--border); border-radius: 16px; overflow: hidden; margin-bottom: 16px; }"
".p-4 { padding: 16px; }"
".item { display: flex; align-items: center; padding: 16px; border-bottom: 1px solid var(--border); cursor: pointer; transition: background 0.2s; }"
".item:active { background: #1e2945; }"
".icon { font-size: 1.4rem; margin-right: 14px; }"
".name { font-weight: 600; font-size: 1rem; }"
".meta { margin-left: auto; color: #94a3b8; font-size: 0.9rem; }"
"button { width: 100%; padding: 14px; border-radius: 12px; border: none; background: var(--primary); color: white; font-weight: 700; font-size: 1rem; cursor: pointer; margin-top: 8px; }"
"button.secondary { background: #334155; margin-top: 0; }"
"input { width: 100%; box-sizing: border-box; padding: 14px; background: #0f172a; border: 1px solid var(--border); color: white; border-radius: 12px; font-size: 1rem; margin-bottom: 12px; }"
".grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-top: 12px; }"
".path-bread { font-size: 0.9rem; opacity: 0.7; margin-bottom: 12px; }"
"</style></head><body>"
"<div class='top'><div class='logo'>BSQ Timer</div><div class='status' id='net'>...</div></div>"
"<div class='container'><div class='section-title'>Control</div><div class='card p-4'><div class='path-bread' id='path'>Root</div><div id='timer-ui' style='display:none'><div style='text-align:center;margin-bottom:16px;font-size:1.2rem;font-weight:700' id='timer-name'></div><div id='controls'></div></div><div id='list'></div><div class='grid'><button class='secondary' onclick='goUp()'>Back</button><button class='secondary' onclick='goRoot()'>Home</button></div></div>"
"<div class='section-title'>Settings</div><div class='card p-4'><div style='font-weight:700;margin-bottom:12px'>WiFi Configuration</div><input id='ssid' placeholder='Network Name'><input id='pass' type='password' placeholder='Password'><button onclick='saveWifi()'>Save Settings</button></div></div>"
"<script>"
"let menu=null;let curPath='/';"
"function joinPath(base,name){if(base==='/')return '/'+name;return base+'/'+name;}"
"function parentPath(p){if(p==='/')return '/';const i=p.lastIndexOf('/');return (i<=0)?'/':p.slice(0,i);}"
"function findNode(path){if(!menu)return null;if(path==='/')return menu;const parts=path.split('/').filter(Boolean);let n=menu;for(const part of parts){if(!n.children)return null;n=n.children.find(x=>x.name===part);if(!n)return null;}return n;}"
"function renderList(){const n=findNode(curPath);document.getElementById('path').textContent=curPath==='/'?'Main Menu':curPath;const list=document.getElementById('list');const tUi=document.getElementById('timer-ui');list.innerHTML='';tUi.style.display='none';list.style.display='block';"
"if(!n||!n.children||n.children.length===0){list.innerHTML='<div style=\"padding:16px;opacity:0.6;text-align:center\">Empty Folder</div>';return;}n.children.forEach(ch=>{const d=document.createElement('div');d.className='item';const isF=ch.type==='folder';d.innerHTML=`<div class='icon'>${isF?'📁':'⏲️'}</div><div class='name'>${ch.name}</div><div class='meta'>${isF?'>':''}</div>`;d.onclick=()=>onItem(ch);list.appendChild(d);});}"
"async function api(path,opts){const r=await fetch(path,opts);const txt=await r.text();try{return JSON.parse(txt);}catch(e){return {raw:txt,ok:r.ok};}}"
"async function loadMenu(){menu=await api('/api/menu');renderList();}"
"async function onItem(ch){const p=joinPath(curPath,ch.name);if(ch.type==='folder'){curPath=p;await api('/api/select',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({path:curPath})});renderList();}else{await api('/api/select',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({path:p})});renderTimerPanel(p,ch);}}"
"function goUp(){curPath=parentPath(curPath);api('/api/select',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({path:curPath})});renderList();}"
"function goRoot(){curPath='/';api('/api/select',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({path:curPath})});renderList();}"
"function renderTimerPanel(path,node){const list=document.getElementById('list');const tUi=document.getElementById('timer-ui');const c=document.getElementById('controls');list.style.display='none';tUi.style.display='block';document.getElementById('timer-name').textContent=node.name;c.innerHTML='';if(node.mode==='fixed'){const b=document.createElement('button');b.textContent=`Start (${node.fixed}s)`;b.onclick=()=>startTimer(node.fixed);c.appendChild(b);}else{const inp=document.createElement('input');inp.placeholder='Enter seconds...';inp.id='varsec';inp.type='number';c.appendChild(inp);const b=document.createElement('button');b.textContent='Start Timer';b.onclick=()=>{const s=parseInt(document.getElementById('varsec').value||'0',10);if(s>0)startTimer(s);};c.appendChild(b);}}"
"async function startTimer(sec){await api('/api/user/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({seconds:sec})});}"
"async function saveWifi(){const ssid=document.getElementById('ssid').value.trim();const pass=document.getElementById('pass').value;await api('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid,pass})});alert('Saved. Please reboot.');}"
"async function pollState(){const s=await api('/api/state');document.getElementById('net').textContent=s.wifi_mode||'';}setInterval(pollState,1500);loadMenu();</script></body></html>";

static esp_err_t send_json(httpd_req_t *req, const char *json) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static char* read_body(httpd_req_t *req) {
    int total = req->content_len;
    if (total <= 0 || total > 4096) return NULL;
    char *buf = (char*)calloc(1, total + 1);
    if (!buf) return NULL;
    if (httpd_req_recv(req, buf, total) <= 0) { free(buf); return NULL; }
    buf[total] = 0;
    return buf;
}

// ---------------- HTTP Handlers ----------------

static esp_err_t h_root(httpd_req_t *req) { return httpd_resp_send(req, "<html><a href='/user'>User</a> <a href='/admin'>Admin</a></html>", -1); }
static esp_err_t h_admin(httpd_req_t *req) { wake_screen(); return httpd_resp_send(req, HTML_ADMIN, -1); }
static esp_err_t h_user(httpd_req_t *req) { wake_screen(); return httpd_resp_send(req, HTML_USER, -1); }

static esp_err_t api_menu(httpd_req_t *req) {
    wake_screen();
    if (!g_menu_json) load_menu_from_nvs();
    return send_json(req, g_menu_json);
}

static esp_err_t api_wifi(httpd_req_t *req) {
    wake_screen();
    char *b = read_body(req);
    if (b) {
        cJSON *j = cJSON_Parse(b);
        free(b);
        if (j) {
            cJSON *s = cJSON_GetObjectItem(j,"ssid");
            cJSON *p = cJSON_GetObjectItem(j,"pass");
            if (cJSON_IsString(s)) wifi_save_credentials(s->valuestring, cJSON_IsString(p)?p->valuestring:"");
            cJSON_Delete(j);
        }
    }
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t api_select(httpd_req_t *req) {
    wake_screen();
    char *b = read_body(req);
    if (b) {
        cJSON *j = cJSON_Parse(b);
        free(b);
        if (j) {
            cJSON *p = cJSON_GetObjectItem(j, "path");
            if (cJSON_IsString(p)) {
                strncpy(g_state.selected_path, p->valuestring, sizeof(g_state.selected_path)-1);
                g_state.screen = SCREEN_MENU;
                trigger_ui_update();
            }
            cJSON_Delete(j);
        }
    }
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t api_add(httpd_req_t *req) {
    wake_screen();
    char *b = read_body(req);
    if (b) {
        cJSON *j = cJSON_Parse(b);
        free(b);
        if (j) {
            const char *par="/"; const char *nm=NULL; const char *tp="folder"; const char *md="variable"; int fx=150;
            cJSON *jp=cJSON_GetObjectItem(j,"parent"); if(cJSON_IsString(jp)) par=jp->valuestring;
            cJSON *jn=cJSON_GetObjectItem(j,"name"); if(cJSON_IsString(jn)) nm=jn->valuestring;
            cJSON *jt=cJSON_GetObjectItem(j,"type"); if(cJSON_IsString(jt)) tp=jt->valuestring;
            cJSON *jm=cJSON_GetObjectItem(j,"mode"); if(cJSON_IsString(jm)) md=jm->valuestring;
            cJSON *jf=cJSON_GetObjectItem(j,"fixed"); if(cJSON_IsNumber(jf)) fx=jf->valueint;
            menu_add_item(par, nm, tp, md, fx);
            cJSON_Delete(j);
        }
    }
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t api_del(httpd_req_t *req) {
    wake_screen();
    char *b = read_body(req);
    if (b) {
        cJSON *j = cJSON_Parse(b);
        free(b);
        if (j) {
            cJSON *jp=cJSON_GetObjectItem(j,"parent");
            cJSON *jn=cJSON_GetObjectItem(j,"name");
            if(cJSON_IsString(jp) && cJSON_IsString(jn)) {
                menu_delete_item(jp->valuestring, jn->valuestring);
            }
            cJSON_Delete(j);
        }
    }
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t api_start(httpd_req_t *req) {
    wake_screen();
    char *b = read_body(req);
    if (b) {
        cJSON *j = cJSON_Parse(b);
        free(b);
        if (j) {
            cJSON *s = cJSON_GetObjectItem(j,"seconds");
            if (cJSON_IsNumber(s) && s->valueint > 0) {
                g_state.timer_total = s->valueint;
                g_state.timer_remaining = s->valueint;
                g_state.timer_running = 1;
                // DO NOT draw here. Just set state.
                g_state.screen = SCREEN_TIMER;
                trigger_ui_update();
            }
            cJSON_Delete(j);
        }
    }
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t api_state(httpd_req_t *req) {
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"ok\":true,\"wifi_mode\":\"%s\",\"screen\":%d}", 
             g_state.sta_connected ? "Online" : "Offline", (int)g_state.screen);
    return send_json(req, buf);
}

static httpd_handle_t start_http(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 16;
    httpd_handle_t s = NULL;
    if (httpd_start(&s, &cfg) != ESP_OK) return NULL;

    httpd_uri_t r[] = {
        {"/", HTTP_GET, h_root, NULL},
        {"/admin", HTTP_GET, h_admin, NULL},
        {"/user", HTTP_GET, h_user, NULL},
        {"/api/menu", HTTP_GET, api_menu, NULL},
        {"/api/wifi", HTTP_POST, api_wifi, NULL},
        {"/api/admin/add", HTTP_POST, api_add, NULL},
        {"/api/admin/delete", HTTP_POST, api_del, NULL},
        {"/api/select", HTTP_POST, api_select, NULL},
        {"/api/user/start", HTTP_POST, api_start, NULL},
        {"/api/state", HTTP_GET, api_state, NULL}
    };
    for(int i=0; i<sizeof(r)/sizeof(r[0]); i++) httpd_register_uri_handler(s, &r[i]);
    return s;
}

// ---------------- Wi-Fi Functions ----------------

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "STA disconnected, retry...");
        g_state.sta_connected = false;
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        g_state.sta_connected = true;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        
        // Start services
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "pool.ntp.org");
        esp_sntp_init();

        mdns_init();
        mdns_hostname_set("bsq");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    }
}

static void start_softap(void) {
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap = {0};
    strncpy((char*)ap.ap.ssid, SOFTAP_SSID, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(SOFTAP_SSID);
    strncpy((char*)ap.ap.password, SOFTAP_PASS, sizeof(ap.ap.password));
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "SoftAP started");
}

static void start_sta(const char *ssid, const char *pass) {
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    
    wifi_config_t stacfg = {0};
    strncpy((char*)stacfg.sta.ssid, ssid, sizeof(stacfg.sta.ssid));
    if (pass) strncpy((char*)stacfg.sta.password, pass, sizeof(stacfg.sta.password));
    stacfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &stacfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ---------------- Main ----------------

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_event_group = xEventGroupCreate();

    load_menu_from_nvs();
    oled_init();

    // WiFi Logic
    char *ssid = NULL, *pass = NULL;
    wifi_load_credentials(&ssid, &pass);
    
    if (ssid && strlen(ssid) > 0) {
        ESP_LOGI(TAG, "Connecting to STA: %s", ssid);
        start_sta(ssid, pass);
        // Wait up to 5s
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(5000));
        if (!(xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT)) {
             ESP_LOGW(TAG, "STA failed to connect quickly. It will keep trying in background.");
        }
    } else {
        start_softap();
    }
    
    // Proper Cleanup
    if (ssid) free(ssid);
    if (pass) free(pass);

    start_http();
    xTaskCreate(ui_task, "ui_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "System Started. http://bsq.local/admin");
}