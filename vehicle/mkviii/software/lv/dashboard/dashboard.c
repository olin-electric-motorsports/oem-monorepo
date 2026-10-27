#ifndef _DEFAULT_SOURCE
  #define _DEFAULT_SOURCE /* needed for usleep() */
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#ifdef _MSC_VER
#include <Windows.h>
#else
#include <unistd.h>
#include <pthread.h>
#endif
#include "lvgl/lvgl.h"
#include <gpiod.h>

#if LV_USE_OS != LV_OS_FREERTOS

#define BATTERY_BAR_WIDTH 80
#define BATTERY_BAR_HEIGHT 480
#define BATTERY_SECTIONS 24
#define MAX_SLOGANS 128
#define MAX_LEN 128
#define SLOGAN_FILE "src/slogans.txt"
#define USED_FILE "src/slogan_flags.bin"

// GPIO Pin definitions (BCM numbering)
#define CLK_PIN 17
#define DT_PIN 27
#define SW_PIN 22

// GPIO chip and lines
static struct gpiod_chip *chip;
static struct gpiod_line_request *line_request;
static unsigned int clk_offset = CLK_PIN;
static unsigned int dt_offset = DT_PIN;
static unsigned int sw_offset = SW_PIN;

// Encoder state variables
static int clk_last_state = 0;
static double last_rotation_time = 0;
static double last_mode_change_time = 0;
static int sw_last_state = 1;
static const double MIN_ROTATION_INTERVAL = 0.005;  // 5ms debounce
static const double MODE_CONFIRM_DELAY = 0.5;  // 500ms confirmation delay

// Mode management
static const char* modes[] = {"MENU", "RACE", "QUAL", "PITL"};
static int current_mode_index = 0;
static int pending_mode_index = 0;
static int mode_confirmed = 1;

// Screen state management
typedef enum {
    SCREEN_LOGO,
    SCREEN_DASH,
    SCREEN_ERROR
} screen_state_t;
static screen_state_t current_screen = SCREEN_LOGO;

// Start up screen
static lv_obj_t *logo, *mark, *slogan;

// Speed and tire temperatures
static lv_obj_t *speed, *fl_temp, *fr_temp, *rl_temp, *rr_temp;
static lv_obj_t *fl_border, *fr_border, *rl_border, *rr_border;

// Mode display
static lv_obj_t *mode_text_border, *mode_text, *mode_border, *mode;

// Lap timing
static lv_obj_t *lap_time, *last_time, *best_time;

// Battery
static lv_obj_t *battery_bar, *batt_border, *batt_text;
static lv_obj_t *batt_percent_border, *batt_percent;
static lv_obj_t *batt_temp_border, *batt_temp;
static lv_obj_t *batt_volt_border, *batt_volt;

// Temperature and voltage
static lv_obj_t *temp_border, *temp, *volt_border, *volt;

// Low voltage, high voltage, and RTD
static lv_obj_t *lv_border, *lv, *hv_border, *hv;
static lv_obj_t *rtd_border, *rtd;

// Settings
static lv_obj_t *set_screen, *set_text;

// Driver messages
static lv_obj_t *msg_border, *msg;

// Throttle and brake
static lv_obj_t *throttle_cont, *throttle, *throttle_text;
static lv_obj_t *brake_cont, *brake, *brake_text;

// Error and safety systems
static lv_obj_t *error, *ts, *ams, *imd, *error_msg, *error_msg_border;
static lv_timer_t *lap_timer;

// Misc
static uint32_t lap_start_ms;
static char slogans[MAX_SLOGANS][MAX_LEN];
static int slogan_count = 0;
static unsigned char used[MAX_SLOGANS];
static lv_timer_t *mode_confirm_timer = NULL;

static double get_time_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static uint32_t get_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void setup_gpio(void) {
    struct gpiod_request_config *req_cfg;
    struct gpiod_line_config *line_cfg;
    struct gpiod_line_settings *settings;
    unsigned int offsets[3] = {CLK_PIN, DT_PIN, SW_PIN};
    
    chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {
        fprintf(stderr, "Failed to open GPIO chip\n");
        fprintf(stderr, "Try running with sudo or check GPIO permissions\n");
        exit(1);
    }
    
    req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
        fprintf(stderr, "Failed to create request config\n");
        gpiod_chip_close(chip);
        exit(1);
    }
    gpiod_request_config_set_consumer(req_cfg, "rotary_encoder");
    
    settings = gpiod_line_settings_new();
    if (!settings) {
        fprintf(stderr, "Failed to create line settings\n");
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        exit(1);
    }
    
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);
    
    line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        fprintf(stderr, "Failed to create line config\n");
        gpiod_line_settings_free(settings);
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        exit(1);
    }
    
    gpiod_line_config_add_line_settings(line_cfg, offsets, 3, settings);
    
    line_request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    
    gpiod_line_settings_free(settings);
    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    
    if (!line_request) {
        fprintf(stderr, "Failed to request GPIO lines\n");
        fprintf(stderr, "Make sure GPIO pins are not in use\n");
        gpiod_chip_close(chip);
        exit(1);
    }
    
    clk_last_state = gpiod_line_request_get_value(line_request, clk_offset);
    sw_last_state = gpiod_line_request_get_value(line_request, sw_offset);
}

static void update_mode_label(void) {
    lv_label_set_text(mode, modes[pending_mode_index]);
}

static void hide_set_screen_cb(lv_timer_t *timer)
{
    lv_obj_add_flag(set_screen, LV_OBJ_FLAG_HIDDEN);
    lv_timer_delete(timer);
}

static void mode_confirm_timer_cb(lv_timer_t *timer) {
    // Flash the screen with the confirmed mode
    lv_obj_remove_flag(set_screen, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(set_text, modes[pending_mode_index]);
    
    // Confirm the mode change
    current_mode_index = pending_mode_index;
    mode_confirmed = 1;
    
    // Hide the flash screen after 1 second
    lv_timer_create(hide_set_screen_cb, 1000, NULL);
    
    // Delete this timer
    mode_confirm_timer = NULL;
    lv_timer_delete(timer);
}

static void handle_mode_change(int direction) {
    // Update pending mode
    pending_mode_index += direction;
    if (pending_mode_index < 0) pending_mode_index = 3;
    if (pending_mode_index > 3) pending_mode_index = 0;
    
    // Update the label immediately
    update_mode_label();
    
    // Reset confirmation state
    mode_confirmed = 0;
    last_mode_change_time = get_time_seconds();
    
    // Cancel existing timer if any
    if (mode_confirm_timer != NULL) {
        lv_timer_delete(mode_confirm_timer);
        mode_confirm_timer = NULL;
    }
    
    // Create new confirmation timer
    mode_confirm_timer = lv_timer_create(mode_confirm_timer_cb, (uint32_t)(MODE_CONFIRM_DELAY * 1000), NULL);
}

static void read_encoder(void) {
    double current_time = get_time_seconds();
    int clk_state = gpiod_line_request_get_value(line_request, clk_offset);
    
    if (current_screen == SCREEN_DASH) {
        if (clk_state == 0 && clk_last_state == 1) {
            if ((current_time - last_rotation_time) > MIN_ROTATION_INTERVAL) {
                int dt_state = gpiod_line_request_get_value(line_request, dt_offset);
                if (dt_state == 0) {
                    handle_mode_change(-1);
                } else {
                    handle_mode_change(1);
                }
                last_rotation_time = current_time;
            }
        }
    }
    
    clk_last_state = clk_state;
}

static void hide_logo_screen(void) {
    if (logo != NULL) lv_obj_add_flag(logo, LV_OBJ_FLAG_HIDDEN);
    if (mark != NULL) lv_obj_add_flag(mark, LV_OBJ_FLAG_HIDDEN);
    if (slogan != NULL) lv_obj_add_flag(slogan, LV_OBJ_FLAG_HIDDEN);
}

static void show_logo_screen(void) {
    if (logo != NULL) lv_obj_remove_flag(logo, LV_OBJ_FLAG_HIDDEN);
    if (mark != NULL) lv_obj_remove_flag(mark, LV_OBJ_FLAG_HIDDEN);
    if (slogan != NULL) lv_obj_remove_flag(slogan, LV_OBJ_FLAG_HIDDEN);
}

static void hide_dash_elements(void) {
    lv_obj_t *objs[] = {
        speed, fl_border, fr_border, rl_border, rr_border,
        fl_temp, fr_temp, rl_temp, rr_temp, mode_text_border,
        mode_border, lap_time, last_time, best_time,
        battery_bar, rtd_border, batt_border, batt_percent_border,
        lv_border, hv_border, batt_temp_border, temp_border,
        batt_volt_border, volt_border, throttle_cont,
        throttle_text, brake_cont, brake_text, msg_border
    };
    for(size_t i = 0; i < sizeof(objs)/sizeof(objs[0]); i++) {
        lv_obj_add_flag(objs[i], LV_OBJ_FLAG_HIDDEN);
    }
}

static void show_dash_elements(void) {
    lv_obj_t *objs[] = {
        speed, fl_border, fr_border, rl_border, rr_border,
        fl_temp, fr_temp, rl_temp, rr_temp, mode_text_border,
        mode_border, lap_time, last_time, best_time,
        battery_bar, rtd_border, batt_border, batt_percent_border,
        lv_border, hv_border, batt_temp_border, temp_border,
        batt_volt_border, volt_border, throttle_cont,
        throttle_text, brake_cont, brake_text, msg_border
    };
    for(size_t i = 0; i < sizeof(objs)/sizeof(objs[0]); i++) {
        lv_obj_remove_flag(objs[i], LV_OBJ_FLAG_HIDDEN);
    }
}

static void hide_error_elements(void) {
    lv_obj_t * objs[] = {
        error, ts, ams, imd, error_msg, error_msg_border
    };
    for(size_t i = 0; i < sizeof(objs)/sizeof(objs[0]); i++) {
        lv_obj_add_flag(objs[i], LV_OBJ_FLAG_HIDDEN);
    }
}

static void show_error_elements(void) {
    lv_obj_t * objs[] = {
        error, ts, ams, imd, error_msg, error_msg_border
    };
    for(size_t i = 0; i < sizeof(objs)/sizeof(objs[0]); i++) {
        lv_obj_remove_flag(objs[i], LV_OBJ_FLAG_HIDDEN);
    }
}

static void lap_timer_cb(lv_timer_t *timer) {
    lv_obj_t *label = lv_timer_get_user_data(timer);
    uint32_t elapsed = get_ms() - lap_start_ms;
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%02lu:%02lu.%03lu", elapsed/60000, (elapsed%60000)/1000, elapsed%1000);
    lv_label_set_text(label, buf);
}

static void switch_to_screen(screen_state_t new_screen) {
    // Hide all screens first
    hide_logo_screen();
    hide_dash_elements();
    hide_error_elements();
    
    // Show the requested screen
    switch(new_screen) {
        case SCREEN_LOGO:
            show_logo_screen();
            break;
        case SCREEN_DASH:
            show_dash_elements();
            if (lap_timer == NULL) {
                lap_start_ms = get_ms();
                lap_timer = lv_timer_create(lap_timer_cb, 10, lap_time);
            }
            break;
        case SCREEN_ERROR:
            show_error_elements();
            break;
    }
    
    current_screen = new_screen;
}

static void handle_button_press(void) {
    // Cycle through screens: ERROR -> DASH -> LOGO -> ERROR
    switch(current_screen) {
        case SCREEN_ERROR:
            switch_to_screen(SCREEN_DASH);
            break;
        case SCREEN_DASH:
            switch_to_screen(SCREEN_LOGO);
            break;
        case SCREEN_LOGO:
            switch_to_screen(SCREEN_ERROR);
            break;
    }
}

static void read_button(void) {
    int sw_state = gpiod_line_request_get_value(line_request, sw_offset);
    
    if (sw_state == 0 && sw_last_state == 1) {
        handle_button_press();
        usleep(50000);
    }
    
    sw_last_state = sw_state;
}

static void load_slogans() {
    FILE *f = fopen(SLOGAN_FILE, "r");
    if (!f) exit(1);
    slogan_count = 0;
    while(fgets(slogans[slogan_count], MAX_LEN, f)) {
        slogans[slogan_count][strcspn(slogans[slogan_count], "\n")] = 0;
        if (++slogan_count >= MAX_SLOGANS) break;
    }
    fclose(f);
}

static void load_used_flags() {
    FILE *f = fopen(USED_FILE, "rb");
    if (!f) { memset(used, 0, sizeof(used)); return; }
    fread(used, 1, slogan_count, f);
    fclose(f);
}

static void save_used_flags() {
    FILE *f = fopen(USED_FILE, "wb");
    fwrite(used, 1, slogan_count, f);
    fclose(f);
}

static int pick_random_unused() {
    int count = 0;
    for(int i=0;i<slogan_count;i++) if(!used[i]) count++;
    if(count==0) { memset(used,0,slogan_count); save_used_flags(); count=slogan_count; }
    int target = rand()%count;
    for(int i=0;i<slogan_count;i++) if(!used[i] && target--==0) return i;
    return 0;
}

static void wrap_slogan(char *s) {
    int len = strlen(s); if(len<=45) return;
    int mid=len/2, best=-1, best_dist=9999;
    for(int i=0;i<len;i++) if(s[i]==',' && abs(i-mid)<best_dist){best=i; break;}
    if(best!=-1){memmove(&s[best+2],&s[best+1],strlen(&s[best+1])+1); s[best+1]='\n'; return;}
    best_dist=9999; best=-1;
    for(int i=0;i<len;i++) if(s[i]==' ' && abs(i-mid)<best_dist){best=i; best_dist=abs(i-mid);}
    if(best!=-1) s[best]='\n';
}

static lv_color_t get_tire_color(int temp) {
    if(temp<0) temp=0; 
    if(temp>200) temp=200;
    if(temp<=40) return lv_color_hex(0xC8C8C8);
    if(temp<=70){float t=(temp-40)/30.0f; return lv_color_make((uint8_t)(200*(1-t)),(uint8_t)(200+55*t),(uint8_t)(200*(1-t)));}
    if(temp<=90) return lv_color_hex(0x00FF00);
    if(temp<=120){float t=(temp-90)/30.0f; return lv_color_make((uint8_t)(255*t),(uint8_t)(255*(1-t)),0);}
    return lv_color_hex(0xFF0000);
}

static void update_tire_color(lv_obj_t *border,int temp){lv_obj_set_style_bg_color(border,get_tire_color(temp),LV_PART_MAIN);}

static void update_all_tire_colors(int fl,int fr,int rl,int rr){update_tire_color(fl_border,fl);update_tire_color(fr_border,fr);update_tire_color(rl_border,rl);update_tire_color(rr_border,rr);}

static void tire_color_timer(lv_timer_t *timer){
    int fl=atoi(lv_label_get_text(fl_temp)),fr=atoi(lv_label_get_text(fr_temp)),rl=atoi(lv_label_get_text(rl_temp)),rr=atoi(lv_label_get_text(rr_temp));
    update_all_tire_colors(fl,fr,rl,rr);

    lv_timer_delete(timer);
}

static lv_color_t get_battery_color(int index,int total){
    float t=(float)index/(total-1); uint8_t r,g,b=0;
    if(t<=0.25f){float lt=t/0.25f; r=255; g=(lt<0.5f)?(uint8_t)(lt/0.5f*120):(uint8_t)(120+(lt-0.5f)/0.5f*(200-120));}
    else{float lt=(t-0.25f)/0.75f; r=(uint8_t)((1-lt)*255); g=(uint8_t)(200+lt*(255-200));}
    return lv_color_make(r,g,b);
}

static void update_battery_bar(int percentage){
    lv_obj_clean(battery_bar); if(percentage<0) percentage=0; if(percentage>100) percentage=100;
    int h=BATTERY_BAR_HEIGHT/BATTERY_SECTIONS,filled=(percentage*BATTERY_SECTIONS+99)/100;
    for(int i=0;i<BATTERY_SECTIONS;i++){
        lv_obj_t *s=lv_obj_create(battery_bar);
        lv_obj_remove_flag(s,LV_OBJ_FLAG_SCROLLABLE); lv_obj_set_style_radius(s,0,LV_PART_MAIN);
        lv_obj_set_size(s,BATTERY_BAR_WIDTH,h); lv_obj_align(s,LV_ALIGN_BOTTOM_MID,0,-i*h+22);
        lv_obj_set_style_bg_color(s,(i<filled)?get_battery_color(i,BATTERY_SECTIONS):lv_color_black(),LV_PART_MAIN);
        lv_obj_set_style_border_width(s,1,LV_PART_MAIN);
    }
}

static void msg_enable_vertical_scroll(lv_obj_t * msg, lv_coord_t view_height)
{
    lv_coord_t text_h = lv_obj_get_height(msg);

    if(text_h <= view_height) {
        return;
    }

    lv_obj_align(msg, LV_ALIGN_TOP_MID, 0, 0);

    lv_anim_t a;
    lv_anim_init(&a);

    lv_anim_set_var(&a, msg);

    lv_anim_set_time(&a, text_h*15);             // scroll up
    lv_anim_set_playback_time(&a, text_h*15);    // scroll down

    lv_anim_set_playback_delay(&a, 1000);    // pause at BOTTOM
    lv_anim_set_repeat_delay(&a, 1000);      // pause at TOP

    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);

    lv_anim_set_values(&a, 0, -(text_h - view_height));

    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_start(&a);
}

static lv_obj_t* create_label(lv_obj_t *parent,const char *txt,lv_color_t color,const lv_font_t *font,int hidden){
    lv_obj_t *lbl=lv_label_create(parent);
    if(hidden) lv_obj_add_flag(lbl,LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(lbl,txt);
    lv_obj_set_style_text_color(lbl,color,LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl,font,LV_PART_MAIN);
    return lbl;
}

static lv_obj_t* create_border(lv_obj_t *parent,int w,int h,lv_color_t bg,lv_color_t border){
    lv_obj_t *b=lv_obj_create(parent);
    lv_obj_remove_flag(b,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(b,w,h);
    lv_obj_set_style_radius(b,0,LV_PART_MAIN);
    lv_obj_set_style_border_width(b,0,LV_PART_MAIN);
    lv_obj_set_style_border_color(b,border,LV_PART_MAIN);
    lv_obj_set_style_bg_color(b,bg,LV_PART_MAIN);
    return b;
}

static void delete_logo(lv_timer_t *timer)
{
    lv_obj_t *objs[] = {logo, mark, slogan};
    for(size_t i = 0; i < sizeof(objs)/sizeof(objs[0]); i++) {
        lv_obj_delete(objs[i]);
    }
    lv_timer_delete(timer);
}

static void change_speed(lv_timer_t *timer)
{
    lv_label_set_text(speed, "26");
    update_battery_bar(76);
    lv_label_set_text(batt_percent, "76.0%");
    lv_obj_set_style_bg_color(rtd_border, lv_color_hex(0x00ff00), LV_PART_MAIN);
    lv_obj_set_style_bg_color(hv_border, lv_color_hex(0x00ff00), LV_PART_MAIN);
    lv_bar_set_value(throttle, 68, LV_ANIM_OFF);
    lv_label_set_text(throttle_text, "68");
    lv_bar_set_value(brake, 23, LV_ANIM_OFF);
    lv_label_set_text(brake_text, "23");
    lv_timer_delete(timer);
}

static void set_mode(lv_timer_t *timer)
{
    lv_obj_remove_flag(set_screen, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(mode, "QUAL");
    lv_timer_create(hide_set_screen_cb, 1000, NULL);
    lv_timer_delete(timer);
}

static void show_dash(lv_timer_t *timer)
{
    switch_to_screen(SCREEN_DASH);
    lv_timer_delete(timer);
}

static void hide_dash(lv_timer_t *timer)
{
    hide_dash_elements();
    lv_timer_delete(timer);
}

static void show_error(lv_timer_t *timer)
{
    switch_to_screen(SCREEN_ERROR);
    lv_timer_delete(timer);
}

static void hide_error(lv_timer_t *timer)
{
    hide_error_elements();
    lv_timer_delete(timer);
}

int main(int argc,char **argv){
    /* Initialize LVGL */
    lv_init();

    /* Initialize GPIO */
    setup_gpio();

    /* Add framebuffer display setup */
    lv_display_t *disp = lv_linux_fbdev_create();
    lv_linux_fbdev_set_file(disp, "/dev/fb0");
    
    (void)argc;(void)argv; srand(time(NULL));

    load_slogans(); load_used_flags();
    int idx=pick_random_unused(); used[idx]=1; save_used_flags();
    char wrapped[MAX_LEN]; strncpy(wrapped,slogans[idx],MAX_LEN-1); wrapped[MAX_LEN-1]=0; wrap_slogan(wrapped);
    lv_obj_set_style_bg_color(lv_screen_active(),lv_color_black(),LV_PART_MAIN);

    logo=lv_image_create(lv_screen_active()); LV_IMAGE_DECLARE(oem_logo); lv_image_set_src(logo,&oem_logo); lv_obj_align(logo,LV_ALIGN_CENTER,0,-40);
    mark=create_label(lv_screen_active(),"Mk VIII",lv_color_hex(0xffffff),&lv_font_montserrat_32,0); lv_obj_align(mark,LV_ALIGN_CENTER,0,50);
    slogan=create_label(lv_screen_active(),wrapped,lv_color_hex(0xffffff),&lv_font_montserrat_28,0); lv_obj_set_style_text_align(slogan,LV_TEXT_ALIGN_CENTER,LV_PART_MAIN); lv_obj_align(slogan,LV_ALIGN_CENTER,0,140);

    speed=create_label(lv_screen_active(),"0",lv_color_hex(0xffffff),&lv_font_roboto_184,1); lv_obj_align(speed,LV_ALIGN_CENTER,0,-80);

    fl_border=create_border(lv_screen_active(),100,72,lv_color_hex(0x00FF00),lv_color_hex(0x000000)); lv_obj_add_flag(fl_border,LV_OBJ_FLAG_HIDDEN); lv_obj_align(fl_border,LV_ALIGN_TOP_LEFT,5,100);
    fr_border=create_border(lv_screen_active(),100,72,lv_color_hex(0x00FF00),lv_color_hex(0x000000)); lv_obj_add_flag(fr_border,LV_OBJ_FLAG_HIDDEN); lv_obj_align(fr_border,LV_ALIGN_TOP_LEFT,110,100);
    rl_border=create_border(lv_screen_active(),100,72,lv_color_hex(0x00FF00),lv_color_hex(0x000000)); lv_obj_add_flag(rl_border,LV_OBJ_FLAG_HIDDEN); lv_obj_align(rl_border,LV_ALIGN_TOP_LEFT,5,177);
    rr_border=create_border(lv_screen_active(),100,72,lv_color_hex(0x00FF00),lv_color_hex(0x000000)); lv_obj_add_flag(rr_border,LV_OBJ_FLAG_HIDDEN); lv_obj_align(rr_border,LV_ALIGN_TOP_LEFT,110,177);

    fl_temp=create_label(fl_border,"80",lv_color_hex(0x000000),&lv_font_roboto_48,1); lv_obj_center(fl_temp);
    fr_temp=create_label(fr_border,"80",lv_color_hex(0x000000),&lv_font_roboto_48,1); lv_obj_center(fr_temp);
    rl_temp=create_label(rl_border,"80",lv_color_hex(0x000000),&lv_font_roboto_48,1); lv_obj_center(rl_temp);
    rr_temp=create_label(rr_border,"80",lv_color_hex(0x000000),&lv_font_roboto_48,1); lv_obj_center(rr_temp);

    mode_text_border=create_border(lv_screen_active(),145,32,lv_color_hex(0x000000),lv_color_hex(0xffffff)); lv_obj_add_flag(mode_text_border,LV_OBJ_FLAG_HIDDEN); lv_obj_align(mode_text_border,LV_ALIGN_BOTTOM_MID,12,-5);
    lv_obj_set_style_border_width(mode_text_border,1,LV_PART_MAIN);
    mode_text=create_label(mode_text_border,"DRIVEMODE",lv_color_hex(0xffffff),&lv_font_roboto_24,0); lv_obj_center(mode_text);

    mode_border=create_border(lv_screen_active(),70,32,lv_color_hex(0xffffff),lv_color_hex(0xffffff)); lv_obj_add_flag(mode_border,LV_OBJ_FLAG_HIDDEN); lv_obj_align(mode_border,LV_ALIGN_BOTTOM_MID,117,-5);
    mode=create_label(mode_border,"MENU",lv_color_hex(0x000000),&lv_font_roboto_24,0); lv_obj_center(mode);

    lap_time=create_label(lv_screen_active(),"00:00.000",lv_color_hex(0xffffff),&lv_font_roboto_64,1); lv_obj_align(lap_time,LV_ALIGN_TOP_LEFT,10,10);
    last_time=create_label(lv_screen_active(),"-0.294",lv_color_hex(0x00ff00),&lv_font_roboto_64,1); lv_obj_align(last_time,LV_ALIGN_TOP_RIGHT,-90,10);
    best_time=create_label(lv_screen_active(),"01:25.892",lv_color_hex(0x9D00FF),&lv_font_roboto_40,1); lv_obj_align(best_time,LV_ALIGN_CENTER,0,15);

    battery_bar=create_border(lv_screen_active(),BATTERY_BAR_WIDTH,BATTERY_BAR_HEIGHT,lv_color_black(),lv_color_white()); lv_obj_add_flag(battery_bar,LV_OBJ_FLAG_HIDDEN); lv_obj_align(battery_bar,LV_ALIGN_BOTTOM_RIGHT,0,0);
    lv_obj_set_style_border_width(battery_bar,1,LV_PART_MAIN);

    rtd_border=create_border(lv_screen_active(),208,32,lv_color_hex(0xff0000),lv_color_black()); lv_obj_add_flag(rtd_border,LV_OBJ_FLAG_HIDDEN); lv_obj_align(rtd_border,LV_ALIGN_BOTTOM_LEFT,5,-5);
    rtd=create_label(rtd_border,"READY TO DRIVE",lv_color_hex(0x000000),&lv_font_roboto_24,0); lv_obj_center(rtd);

    batt_border = create_border(lv_screen_active(), 70, 32, lv_color_hex(0x000000), lv_color_black());
    lv_obj_add_flag(batt_border, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(batt_border, LV_ALIGN_BOTTOM_RIGHT, -165, -5);
    batt_text = create_label(batt_border, "BATT", lv_color_hex(0xffffff), &lv_font_roboto_24, 0);
    lv_obj_center(batt_text);

    batt_percent_border = create_border(lv_screen_active(), 80, 32, lv_color_hex(0xffffff), lv_color_black());
    lv_obj_add_flag(batt_percent_border, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(batt_percent_border, LV_ALIGN_BOTTOM_RIGHT, -85, -5);
    batt_percent = create_label(batt_percent_border, "FULL", lv_color_hex(0x000000), &lv_font_roboto_24, 0);
    lv_obj_center(batt_percent);

    lv_border = create_border(lv_screen_active(), 48, 32, lv_color_hex(0x00ff00), lv_color_black());
    lv_obj_add_flag(lv_border, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(lv_border, LV_ALIGN_BOTTOM_LEFT, 218, -5);
    lv = create_label(lv_border, "LV", lv_color_hex(0x000000), &lv_font_roboto_24, 0);
    lv_obj_center(lv);

    hv_border = create_border(lv_screen_active(), 48, 32, lv_color_hex(0xff0000), lv_color_black());
    lv_obj_add_flag(hv_border, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(hv_border, LV_ALIGN_BOTTOM_LEFT, 271, -5);
    hv = create_label(hv_border, "HV", lv_color_hex(0x000000), &lv_font_roboto_24, 0);
    lv_obj_center(hv);

    batt_temp_border = create_border(lv_screen_active(), 70, 32, lv_color_hex(0x000000), lv_color_black());
    lv_obj_add_flag(batt_temp_border, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(batt_temp_border, LV_ALIGN_BOTTOM_RIGHT, -165, -42);
    batt_temp = create_label(batt_temp_border, "TEMP", lv_color_hex(0xffffff), &lv_font_roboto_24, 0);
    lv_obj_center(batt_temp);

    temp_border = create_border(lv_screen_active(), 80, 32, lv_color_hex(0xffffff), lv_color_black());
    lv_obj_add_flag(temp_border, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(temp_border, LV_ALIGN_BOTTOM_RIGHT, -85, -42);
    temp = create_label(temp_border, "143°F", lv_color_hex(0x000000), &lv_font_roboto_24, 0);
    lv_obj_center(temp);

    batt_volt_border = create_border(lv_screen_active(), 70, 32, lv_color_hex(0x000000), lv_color_black());
    lv_obj_add_flag(batt_volt_border, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(batt_volt_border, LV_ALIGN_BOTTOM_RIGHT, -165, -79);
    batt_volt = create_label(batt_volt_border, "VOLT", lv_color_hex(0xffffff), &lv_font_roboto_24, 0);
    lv_obj_center(batt_volt);

    volt_border = create_border(lv_screen_active(), 80, 32, lv_color_hex(0xffffff), lv_color_black());
    lv_obj_add_flag(volt_border, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(volt_border, LV_ALIGN_BOTTOM_RIGHT, -85, -79);
    volt = create_label(volt_border, "432.7", lv_color_hex(0x000000), &lv_font_roboto_24, 0);
    lv_obj_center(volt);

    throttle_cont = lv_obj_create(lv_screen_active());
    lv_obj_add_flag(throttle_cont, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_size(throttle_cont, 30, 160);
    lv_obj_set_style_radius(throttle_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(throttle_cont, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(throttle_cont, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_bg_color(throttle_cont, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_pad_all(throttle_cont, 1, LV_PART_MAIN);
    lv_obj_align(throttle_cont, LV_ALIGN_TOP_RIGHT, -150, 105);

    throttle = lv_bar_create(throttle_cont);
    lv_obj_set_size(throttle, LV_PCT(100), LV_PCT(100));
    lv_bar_set_range(throttle, 0, 100);
    lv_bar_set_value(throttle, 0, LV_ANIM_OFF);
    lv_obj_set_style_border_width(throttle, 0, 0);
    lv_obj_set_style_radius(throttle, 0, 0);
    lv_obj_set_style_radius(throttle, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(throttle, lv_color_hex(0x00FF00), LV_PART_INDICATOR);
    lv_obj_set_style_transform_pivot_y(throttle, 160, LV_PART_INDICATOR);
    lv_obj_set_style_transform_rotation(throttle, 1800, LV_PART_INDICATOR);

    throttle_text = lv_label_create(lv_screen_active());
    lv_obj_add_flag(throttle_text, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(throttle_text, "0");
    lv_obj_set_style_text_color(throttle_text, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(throttle_text, &lv_font_roboto_24, LV_PART_MAIN);
    lv_obj_align(throttle_text, LV_ALIGN_CENTER, 235, -150);

    brake_cont = lv_obj_create(lv_screen_active());
    lv_obj_add_flag(brake_cont, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_size(brake_cont, 30, 160);
    lv_obj_set_style_radius(brake_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(brake_cont, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(brake_cont, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_bg_color(brake_cont, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_pad_all(brake_cont, 1, LV_PART_MAIN);
    lv_obj_align(brake_cont, LV_ALIGN_TOP_RIGHT, -190, 105);

    brake = lv_bar_create(brake_cont);
    lv_obj_set_size(brake, LV_PCT(100), LV_PCT(100));
    lv_bar_set_range(brake, 0, 100);
    lv_bar_set_value(brake, 0, LV_ANIM_OFF);
    lv_obj_set_style_border_width(brake, 0, 0);
    lv_obj_set_style_radius(brake, 0, 0);
    lv_obj_set_style_radius(brake, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(brake, lv_color_hex(0xFF0000), LV_PART_INDICATOR);
    lv_obj_set_style_transform_pivot_y(brake, 160, LV_PART_INDICATOR);
    lv_obj_set_style_transform_rotation(brake, 1800, LV_PART_INDICATOR);

    brake_text = lv_label_create(lv_screen_active());
    lv_obj_add_flag(brake_text, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(brake_text, "0");
    lv_obj_set_style_text_color(brake_text, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(brake_text, &lv_font_roboto_24, LV_PART_MAIN);
    lv_obj_align(brake_text, LV_ALIGN_CENTER, 195, -150);

    lv_obj_set_style_bg_color(throttle_cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_color(throttle_cont, lv_color_white(), LV_PART_MAIN);

    lv_obj_set_style_bg_color(throttle, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(throttle, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(throttle, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(throttle, lv_color_hex(0x00FF00), LV_PART_INDICATOR);
    lv_obj_set_style_radius(throttle, 0, LV_PART_INDICATOR);

    lv_obj_set_style_bg_color(brake_cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_color(brake, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_color(brake, lv_color_hex(0xFF0000), LV_PART_INDICATOR);

    msg_border = create_border(lv_screen_active(), 547, 156, lv_color_hex(0x000000), lv_color_hex(0xffffff));
    lv_obj_set_style_border_width(msg_border, 1, LV_PART_MAIN);
    lv_obj_add_flag(msg_border, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_pad_all(msg_border, 0, LV_PART_MAIN);
    lv_obj_align(msg_border, LV_ALIGN_BOTTOM_LEFT, 5, -42);

    msg = lv_label_create(msg_border);
    lv_obj_set_width(msg, 547);
    lv_label_set_long_mode(msg, LV_LABEL_LONG_WRAP);
    lv_label_set_text(msg, "HEY KEFAN!");
    lv_obj_set_style_text_color(msg, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(msg, &lv_font_roboto_48, LV_PART_MAIN);
    lv_obj_set_style_text_align(msg, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(msg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_update_layout(msg);
    msg_enable_vertical_scroll(msg, 156);

    set_screen = create_border(lv_screen_active(), 800, 480, lv_color_hex(0xffffff), lv_color_black());
    lv_obj_add_flag(set_screen, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(set_screen, LV_ALIGN_CENTER, 0, 0);

    set_text = create_label(set_screen, "QUAL", lv_color_hex(0x000000), &lv_font_roboto_184, 0);
    lv_obj_center(set_text);

    int battery_level = 100; // example
    update_battery_bar(battery_level);

    // Error Screen
    error = create_label(lv_screen_active(), "CRITICAL ERROR", lv_color_hex(0xFF0000), &lv_font_roboto_48, 1);
    lv_obj_align(error, LV_ALIGN_CENTER, 0, -200);

    ts = create_label(lv_screen_active(), "TS", lv_color_hex(0x222222), &lv_font_roboto_48, 1);
    lv_obj_align(ts, LV_ALIGN_CENTER, -250, 200);

    ams = create_label(lv_screen_active(), "AMS", lv_color_hex(0xFF0000), &lv_font_roboto_48, 1);
    lv_obj_align(ams, LV_ALIGN_CENTER, 0, 200);

    imd = create_label(lv_screen_active(), "IMD", lv_color_hex(0x222222), &lv_font_roboto_48, 1);
    lv_obj_align(imd, LV_ALIGN_CENTER, 250, 200);

    error_msg_border = create_border(lv_screen_active(), 800, 300, lv_color_hex(0x000000), lv_color_hex(0xffffff));
    lv_obj_add_flag(error_msg_border, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_pad_all(error_msg_border, 0, LV_PART_MAIN);
    lv_obj_align(error_msg_border, LV_ALIGN_CENTER, 0, 0);

    error_msg = lv_label_create(error_msg_border);
    lv_obj_add_flag(error_msg, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(error_msg, 800);
    lv_label_set_long_mode(error_msg, LV_LABEL_LONG_WRAP);
    lv_label_set_text(error_msg, "OVER VOLTAGE\nBATTERY TEMP\nBSPD TIMEOUT");
    lv_obj_set_style_text_line_space(error_msg, 5, LV_PART_MAIN);
    lv_obj_set_style_text_color(error_msg, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(error_msg, &lv_font_roboto_32, LV_PART_MAIN);
    lv_obj_set_style_text_align(error_msg, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(error_msg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_update_layout(error_msg);
    msg_enable_vertical_scroll(error_msg, 300);

    lv_timer_create(tire_color_timer, 1000, NULL);
    /*lv_timer_create(delete_logo, 2000, NULL);
    lv_timer_create(show_dash, 2001, NULL);
    lv_timer_create(set_mode, 5000, NULL);
    lv_timer_create(change_speed, 7000, NULL);
    lv_timer_create(hide_dash, 10000, NULL);
    lv_timer_create(show_error, 10001, NULL);
    lv_timer_create(hide_error, 20000, NULL);
    lv_timer_create(show_dash, 20001, NULL);*/

    while(1)
    {
        /* Read encoder and button */
        read_encoder();
        read_button();
        /* Periodically call the lv_task handler.
        * It could be done in a timer interrupt or an OS task too.*/
        uint32_t sleep_time_ms = lv_timer_handler();
        if(sleep_time_ms == LV_NO_TIMER_READY)
        {
            sleep_time_ms =  LV_DEF_REFR_PERIOD;
        }
    #ifdef _MSC_VER
        Sleep(sleep_time_ms);
    #else
        usleep(sleep_time_ms * 1000);
    #endif
    }

    gpiod_line_request_release(line_request);
    gpiod_chip_close(chip);
    
    return 0;
}

#endif /* LV_USE_OS != LV_OS_FREERTOS */

