/*
 * Copyright (C) 2018 Microchip Technology Inc.  All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <egt/ui>
#include <memory>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <random>
#include <chrono>
#include <queue>
#include <poll.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>


#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

#define HAVE_LIBPLANES

#ifdef HAVE_LIBPLANES
#include <planes/plane.h>
#include "egt/detail/screen/kmsoverlay.h"
#endif
#include "../src/detail/erawimage.h"
#include "../src/detail/eraw.h"


#define DO_SVG_SERIALIZATION
//#define CAN_DEBUG

#define MEDIA_FILE_PATH "/root/"
#define solve_relative_path(relative_path) (MEDIA_FILE_PATH + relative_path)

#ifdef HAVE_LIBPLANES
#define SCALE_TO_MAX_AND_STAY    //If this macro defined, scale animation will scale from min to max and stay, then enter to demo.
                                 //Else if not defined, scale animation will scale from min to max, and then scale to min, after that enter to demo.
#endif

#define HIGH_Q_TIME_THRESHHOLD 30000  //30ms
#define LOW_Q_TIME_THRESHHOLD  20000  //20ms
//#define EGT_STATISTICS_ENABLE

#ifdef EGT_STATISTICS_ENABLE
#define ID_MAX               1287 //1287 is for testing the random range(110 needles) compare with Cairo draw, the original MAX is 1327
#else
#define ID_MAX               1327
#endif

#define ID_MIN               847
#define STEPPER              4
#define ID_MAX_FUEL          960
#define ID_MIN_FUEL          946
#define FUEL_STEPPER         2
#define ID_MAX_TEMP          983
#define ID_MIN_TEMP          971
#define ID_MAX_TEXT_LEFT     1027
#define ID_MIN_TEXT_LEFT     995
#define ID_MAX_TEXT_RIGHT    1067
#define ID_MIN_TEXT_RIGHT    1031
#define ID_MAX_TEXT_GEAR     1383  //STEPPER=4
#define ID_MIN_TEXT_GEAR     1367
#define ID_MAX_LEFT_SPEED_L  1341  //STEPPER=4
#define ID_MIN_LEFT_SPEED_L  1309
#define ID_MAX_LEFT_SPEED_R  1381  //STEPPER=4
#define ID_MIN_LEFT_SPEED_R  1345
#define ID_MAX_RIGHT_SPEED_L 1417  //STEPPER=4
#define ID_MIN_RIGHT_SPEED_L 1385
#define ID_MAX_RIGHT_SPEED_R 1457  //STEPPER=4
#define ID_MIN_RIGHT_SPEED_R 1421
#define ID_MAX_TEMP_L        1499  //STEPPER=4
#define ID_MIN_TEMP_L        1487
#define ID_MAX_TEMP_R        1539  //STEPPER=4
#define ID_MIN_TEMP_R        1503
#define MAX_TEMP_TABLE       12
#define MAX_SPEED_TABLE      9
#define MAX_DIGIT_PAIR       2
#define MAX_FUEL_REC         8
#define MAX_TEMP_REC         7
#define PLANE_WIDTH_HF       400
#define PLANE_HEIGHT_HF      240
#define MAX_NEEDLE_INDEX     120

int g_digit = ID_MIN;
bool is_increasing = true;
int g_fuel_digit = ID_MIN_FUEL;
bool is_fuel_inc = true;
int g_temp_digit = ID_MAX_TEMP;
bool is_temp_inc = true;
int g_left_txt_digit = ID_MIN_TEXT_LEFT;
int g_right_txt_digit = ID_MIN_TEXT_RIGHT;
bool is_txt_inc = true;
int g_gear_txt_digit = ID_MIN_TEXT_GEAR;
bool is_gear_inc = true;
int g_left_txt_temp = ID_MIN_TEMP_L;
bool is_left_temp_inc = true;
int g_right_txt_temp = ID_MIN_TEMP_R;
bool is_right_temp_inc = true;


enum class SvgEleType
{
    path = 0,
    text,
    bar,
    rbar
};

typedef struct
{
    int main_speed[MAX_DIGIT_PAIR];
    int left_speed[MAX_DIGIT_PAIR];
    int right_speed[MAX_DIGIT_PAIR];
    int speed_digit;
}st_speed_table_t;

typedef struct
{
    int pan_x;
    int pan_y;
    int pan_w;
    int pan_h;

    int x;
    int y;
}st_plane_attri_t;

typedef struct
{
    int y;
    int h;
}st_fuel_temp_t;

#ifndef EGT_STATISTICS_ENABLE
#if 0 //temperature digit use text now
static int g_temp_table[MAX_TEMP_TABLE][MAX_DIGIT_PAIR] =
{
    {1491, 1503}, //40
    {1491, 1507}, //41
    {1491, 1519}, //44
    {1491, 1527}, //46
    {1491, 1535}, //48
    {1491, 1539}, //49

    {1499, 1503}, //60
    {1499, 1507}, //61
    {1499, 1519}, //64
    {1499, 1527}, //66
    {1499, 1535}, //68
    {1499, 1539}  //69
};

//static st_speed_table_t g_speed_table[MAX_SPEED_TABLE] =
static st_speed_table_t g_speed_table[3] =
{
    //{{ 995, 1031}, {1309, 1345}, {1385, 1421}}, //10
    {{ 999, 1035}, {1313, 1349}, {1389, 1425}, 21}, //21
    {{1003, 1039}, {1317, 1353}, {1393, 1429}, 32}, //32
    {{1007, 1043}, {1321, 1357}, {1397, 1433}, 43}  //43
    //{{1011, 1047}, {1325, 1361}, {1401, 1437}}, //54
    //{{1015, 1051}, {1329, 1365}, {1405, 1441}}, //65
    //{{1019, 1055}, {1333, 1369}, {1409, 1445}}, //76
    //{{1023, 1059}, {1337, 1373}, {1413, 1449}}, //87
    //{{1027, 1063}, {1341, 1377}, {1417, 1453}}  //98
};
#endif
#endif

st_plane_attri_t g_fuel[MAX_FUEL_REC] = {
    /* panx, pany, panw, panh, x, y */
    {  0, 0, 23,  22, 17, 328},  //Y bottom=350
    { 23, 0, 23,  44, 17, 306},
    { 46, 0, 23,  66, 17, 284},
    { 69, 0, 23,  88, 17, 262},
    { 92, 0, 23, 110, 17, 240},
    {115, 0, 23, 132, 17, 218},
    {138, 0, 23, 154, 17, 196},
    {161, 0, 23, 175, 17, 175}
};

st_plane_attri_t g_temp[MAX_TEMP_REC] = {
    /* panx, pany, panw, panh, x, y */
    {  0, 0, 23,  24, 757, 324},  //Y bottom=348
    { 23, 0, 23,  51, 757, 297},
    { 46, 0, 23,  77, 757, 271},
    { 69, 0, 23, 103, 757, 245},
    { 92, 0, 23, 129, 757, 219},
    {115, 0, 23, 156, 757, 192},
    {138, 0, 23, 181, 757, 167}
};

st_fuel_temp_t g_fuel_table[7] = {
    {173, 176},
    {194, 155},
    {216, 133},
    {238, 111},
    {259, 90},
    {281, 68},
    {303, 46}
};

st_fuel_temp_t g_temp_table[6] = {
    {324, 25},
    {298, 51},
    {271, 78},
    {245, 104},
    {220, 129},
    {194, 167}
};

// forward reference
void signalI2C();

class MotorDash : public egt::experimental::Gauge
{
public:

#ifdef DO_SVG_SERIALIZATION
    MotorDash(egt::SvgImage& svg) noexcept: m_svg(svg)
#else
    MotorDash() noexcept
#endif
    {
        set_gear_deserial_state(false);
        set_fuel_deserial_state(false);
        set_temp_deserial_state(false);
        set_needle_deserial_state(false);
        set_speed_deserial_state(false);
        set_call_deserial_state(false);
        set_bar_deserial_state(false);
        set_blink_deserial_state(false);
        set_tc_deserial_state(false);
        set_vsc_deserial_state(false);
        set_bat_deserial_state(false);
        set_snow_deserial_state(false);
        set_high_q_state(true);
        set_low_q_state(true);
        set_wifi_deserial_state(false);
    }

#ifdef DO_SVG_SERIALIZATION
    void serialize_all();

    void render_needles()
    {
        for (int i = ID_MIN; i <= ID_MAX; i += STEPPER)
        {
            add_svg_layer_with_digit(m_svg, "#path", i, false);
        }
    }

    void render_needles(int render_start, int render_end)
    {
        for (int i = render_start; i <= render_end; i += STEPPER)
        {
            add_svg_layer_with_digit(m_svg, "#path", i, false);
        }
    }

    void render_needles(int render_digit)
    {
        add_svg_layer_with_digit(m_svg, "#path", render_digit, true);
    }

    void render_left_spd()
    {
        for (int i = ID_MIN_TEXT_LEFT; i <= ID_MAX_TEXT_LEFT; i += STEPPER)
        {
            add_svg_layer_with_digit(m_svg, "#text", i, true);
        }
        //std::cout << "render_left_spd" << std::endl;
    }

    void render_right_spd()
    {
        for (int i = ID_MIN_TEXT_RIGHT; i <= ID_MAX_TEXT_RIGHT; i += STEPPER)
        {
            add_svg_layer_with_digit(m_svg, "#text", i, true);
        }
        //std::cout << "render_right_spd" << std::endl;
    }

    void render_gear()
    {
        //GEAR UI
        for (int i = ID_MIN_TEXT_GEAR; i <= ID_MAX_TEXT_GEAR; i += STEPPER)
        {
            add_svg_layer_with_digit(m_svg, "#text", i, true);
        }
    }

    void render_bar()
    {
        add_svg_layer(m_svg, "#bar0", true);
        add_svg_layer(m_svg, "#rbar0", true);
    }

    void render_bar5()
    {
        add_svg_layer(m_svg, "#bar5", true);
        add_svg_layer(m_svg, "#rbar5", true);
    }

    void render_bar8()
    {
        add_svg_layer(m_svg, "#bar8", true);
        add_svg_layer(m_svg, "#rbar8", true);
    }

    void render_l_spd_l()
    {
        //left speed ID_MIN_LEFT_SPEED_L
        for (int i = ID_MIN_LEFT_SPEED_L; i <= ID_MAX_LEFT_SPEED_L; i += STEPPER)
        {
            add_svg_layer_with_digit(m_svg, "#text", i, true);
        }
    }

    void render_l_spd_r()
    {
        //left speed ID_MIN_LEFT_SPEED_R
        for (int i = ID_MIN_LEFT_SPEED_R; i <= ID_MAX_LEFT_SPEED_R; i += STEPPER)
        {
            add_svg_layer_with_digit(m_svg, "#text", i, true);
        }
    }

    void render_r_spd_l()
    {
        //left speed ID_MIN_RIGHT_SPEED_L
        for (int i = ID_MIN_RIGHT_SPEED_L; i <= ID_MAX_RIGHT_SPEED_L; i += STEPPER)
        {
            add_svg_layer_with_digit(m_svg, "#text", i, true);
        }
    }

    void render_r_spd_r()
    {
        //left speed ID_MIN_RIGHT_SPEED_R
        for (int i = ID_MIN_RIGHT_SPEED_R; i <= ID_MAX_RIGHT_SPEED_R; i += STEPPER)
        {
            add_svg_layer_with_digit(m_svg, "#text", i, true);
        }
    }

    void render_temp_l()
    {
        //left temperature ID_MIN_TEMP_L
        for (int i = ID_MIN_TEMP_L; i <= ID_MAX_TEMP_L; i += STEPPER)
        {
            add_svg_layer_with_digit(m_svg, "#text", i, true);
        }
    }

    void render_temp_r()
    {
        //right temperature ID_MIN_TEMP_R
        for (int i = ID_MIN_TEMP_R; i <= ID_MAX_TEMP_R; i += STEPPER)
        {
            add_svg_layer_with_digit(m_svg, "#text", i, true);
        }
    }
#endif

    void test()
    {
        auto text = std::make_shared<egt::TextBox>("5", egt::Rect(egt::Point(82, 1), egt::Size(20, 2)));
        text->border(0);
        text->font(egt::Font(40, egt::Font::Weight::normal));
        text->color(egt::Palette::ColorId::bg, egt::Palette::transparent);
        text->color(egt::Palette::ColorId::text, egt::Palette::white);
        add(text);
    }

    void add_text_widget(const std::string& id, const std::string& txt, const egt::Rect& rect, egt::Font::Size size)
    {
        auto text = std::make_shared<egt::TextBox>(txt, rect, egt::AlignFlag::center);
        text->border(0);
        text->font(egt::Font(size, egt::Font::Weight::normal));
        text->color(egt::Palette::ColorId::bg, egt::Palette::transparent);
        text->color(egt::Palette::ColorId::text, egt::Palette::white);
        add(text);
        text->name(id);
    }

    void add_text_widget(const std::string& id, const std::string& txt, const egt::Rect& rect, egt::Font::Size size, egt::Color color)
    {
        auto text = std::make_shared<egt::TextBox>(txt, rect);
        text->border(0);
        text->font(egt::Font(size, egt::Font::Weight::normal));
        text->color(egt::Palette::ColorId::bg, egt::Palette::transparent);
        text->color(egt::Palette::ColorId::text, color);
        add(text);
        text->name(id);
    }

#ifdef DO_SVG_SERIALIZATION
    void add_rec_widget(const std::string& id, bool is_hiding)
    {
        if (!m_svg.id_exists(id))
            return;

        auto box = m_svg.id_box(id);
        auto rec = std::make_shared<egt::RectangleWidget>(egt::Rect(std::floor(box.x()),
                                                                    std::floor(box.y()),
                                                                    std::ceil(box.width()),
                                                                    std::ceil(box.height())));
        rec->color(egt::Palette::ColorId::button_bg, egt::Palette::black);
        if (is_hiding)
            rec->hide();
        add(rec);
        rec->name(id);
    }
#endif

    void add_rec_widget_with_rec(const std::string& id, const egt::Rect& rect, bool is_hiding)
    {
        auto rec = std::make_shared<egt::RectangleWidget>(rect);
        rec->color(egt::Palette::ColorId::button_bg, egt::Palette::black);
        if (is_hiding)
            rec->hide();
        add(rec);
        rec->name(id);
    }

    std::shared_ptr<egt::TextBox> find_text(const std::string& name)
    {
        return find_child<egt::TextBox>(name);
    }

    std::shared_ptr<egt::RectangleWidget> find_rec(const std::string& name)
    {
        return find_child<egt::RectangleWidget>(name);
    }

    std::shared_ptr<egt::experimental::GaugeLayer> find_layer(const std::string& name)
    {
        return find_child<egt::experimental::GaugeLayer>(name);
    }

    void set_text(const std::string& id, const std::string& text, int font_size, const egt::Pattern& color)
    {
        auto layer = find_rec(id);
        auto ptext = std::make_shared<egt::Label>();
        ptext->text_align(egt::AlignFlag::center);
        ptext->box(egt::Rect(layer->box().x(), layer->box().y(), layer->box().width(), layer->box().height()));
        ptext->color(egt::Palette::ColorId::label_text, color);
        ptext->font(egt::Font(font_size, egt::Font::Weight::normal));
        ptext->text(text);
        add(ptext);
    }

    void set_color(const egt::Color& color)
    {
        for (auto i = ID_MIN; i <= ID_MAX; i += STEPPER)
        {
            std::ostringstream ss;
            ss << "#path" << std::to_string(i);
            find_layer(ss.str())->mask_color(color);
        }
    }

    void apply(int digit, bool is_inc, SvgEleType type)
    {
        //std::cout << "apply: " << digit << "  type:" << static_cast<int>(type) << "  inc:" << is_inc << std::endl;
        std::ostringstream ss;
        switch (type)
        {
            case SvgEleType::path:
                ss << "#path" << std::to_string(digit);
                break;
            case SvgEleType::text:
                ss << "#text" << std::to_string(digit);
                find_layer(ss.str())->show();
                return;
            case SvgEleType::bar:
                ss << "#bar" << std::to_string(digit);
                find_layer(ss.str())->show();
                return;
            case SvgEleType::rbar:
                ss << "#rbar" << std::to_string(digit);
                find_layer(ss.str())->show();
                return;
            default:
                break;
        }

        if (is_inc)
        {
            //std::cout << "show: " << ss.str() << std::endl;
            find_layer(ss.str())->show();
        }
        else
        {
            //std::cout << "hide: " << ss.str() << std::endl;
            find_layer(ss.str())->hide();
        }
    }

    void show_digit(int digit)
    {
        std::ostringstream ss;
        ss << "#path" << std::to_string(digit);
        //std::cout << "show_digit: " << ss.str() << std::endl;
        find_layer(ss.str())->show();
    }

    void hide_some(int start, int loop)
    {
        for (int i=0; i < loop; i++)
        {
            std::ostringstream ss;
            ss << "#path" << std::to_string(start);
            //std::cout << "hide_some: " << ss.str() << std::endl;
            find_layer(ss.str())->hide();
            start += STEPPER;
        }
    }

    void hide_speed_text()
    {
        for (int i=ID_MIN_TEXT_LEFT; i <= ID_MAX_TEXT_RIGHT; i += STEPPER)
        {
            std::ostringstream ss;
            ss << "#text" << std::to_string(i);
            find_layer(ss.str())->hide();
        }
    }

    void hide_left_speed_text()
    {
        for (int i=ID_MIN_LEFT_SPEED_L; i <= ID_MAX_LEFT_SPEED_R; i += STEPPER)
        {
            std::ostringstream ss;
            ss << "#text" << std::to_string(i);
            find_layer(ss.str())->hide();
        }
    }

    void hide_right_speed_text()
    {
        for (int i=ID_MIN_RIGHT_SPEED_L; i <= ID_MAX_RIGHT_SPEED_R; i += STEPPER)
        {
            std::ostringstream ss;
            ss << "#text" << std::to_string(i);
            find_layer(ss.str())->hide();
        }
    }

    void hide_all_needles(std::vector<std::shared_ptr<egt::experimental::GaugeLayer>> NeedleBase)
    {
        for (int i = 0; i <= MAX_NEEDLE_INDEX; i++)
        {
            NeedleBase[i]->hide();
        }
    }

    void hide_gear_text(std::vector<std::shared_ptr<egt::experimental::GaugeLayer>> GearBase)
    {
        for (int i = 0; i <= 4; i++)
        {
            GearBase[i]->hide();
        }
    }

    void hide_fuel_rect(std::vector<std::shared_ptr<egt::experimental::GaugeLayer>> FuelBase)
    {
        for (int i = 0; i <= 7; i++)
        {
            FuelBase[i]->hide();
        }
    }

    void hide_temp_rect(std::vector<std::shared_ptr<egt::experimental::GaugeLayer>> TempBase)
    {
        for (int i = 0; i <= 6; i++)
        {
            TempBase[i]->hide();
        }
    }

    void hide_bar_text()
    {
        find_layer("#bar5")->hide();
        find_layer("#rbar5")->hide();
        find_layer("#bar8")->hide();
        find_layer("#rbar8")->hide();
    }

    void hide_all()
    {
        for (auto& child : m_children)
        {
            if (child->name().rfind("#path", 0) == 0)
                child->hide();
        }
    }

    bool get_high_q_state() { return m_is_high_q_quit; }
    void set_high_q_state(bool is_high_q_quit) { m_is_high_q_quit = is_high_q_quit; }

    bool get_low_q_state() { return m_is_low_q_quit; }
    void set_low_q_state(bool is_low_q_quit) { m_is_low_q_quit = is_low_q_quit; }

    bool get_gear_deserial_state() { return m_gear_deserial_finish; }
    void set_gear_deserial_state(bool gear_deserial_finish) { m_gear_deserial_finish = gear_deserial_finish; }

    bool get_fuel_deserial_state() { return m_fuel_deserial_finish; }
    void set_fuel_deserial_state(bool fuel_deserial_finish) { m_fuel_deserial_finish = fuel_deserial_finish; }

    bool get_temp_deserial_state() { return m_temp_deserial_finish; }
    void set_temp_deserial_state(bool temp_deserial_finish) { m_temp_deserial_finish = temp_deserial_finish; }

    bool get_needle_deserial_state() { return m_needle_deserial_finish; }
    void set_needle_deserial_state(bool needle_deserial_finish) { m_needle_deserial_finish = needle_deserial_finish; }

    bool get_speed_deserial_state() { return m_speed_deserial_finish; }
    void set_speed_deserial_state(bool speed_deserial_finish) { m_speed_deserial_finish = speed_deserial_finish; }

    bool get_call_deserial_state() { return m_call_deserial_finish; }
    void set_call_deserial_state(bool call_deserial_finish) { m_call_deserial_finish = call_deserial_finish; }

    bool get_bar_deserial_state() { return m_bar_deserial_finish; }
    void set_bar_deserial_state(bool bar_deserial_finish) { m_bar_deserial_finish = bar_deserial_finish; }

    bool get_blink_deserial_state() { return m_blink_deserial_finish; }
    void set_blink_deserial_state(bool blink_deserial_finish) { m_blink_deserial_finish = blink_deserial_finish; }

    bool get_tc_deserial_state() { return m_tc_deserial_finish; }
    void set_tc_deserial_state(bool tc_deserial_finish) { m_tc_deserial_finish = tc_deserial_finish; }

    bool get_vsc_deserial_state() { return m_vsc_deserial_finish; }
    void set_vsc_deserial_state(bool vsc_deserial_finish) { m_vsc_deserial_finish = vsc_deserial_finish; }

    bool get_bat_deserial_state() { return m_bat_deserial_finish; }
    void set_bat_deserial_state(bool bat_deserial_finish) { m_bat_deserial_finish = bat_deserial_finish; }

    bool get_snow_deserial_state() { return m_snow_deserial_finish; }
    void set_snow_deserial_state(bool snow_deserial_finish) { m_snow_deserial_finish = snow_deserial_finish; }

    bool get_wifi_deserial_state() { return m_wifi_deserial_finish; }
    void set_wifi_deserial_state(bool wifi_deserial_finish) { m_wifi_deserial_finish = wifi_deserial_finish; }

    //egt::Application *app_ptr = nullptr;
    int needle_digit = ID_MIN;

    egt::shared_cairo_surface_t DeSerialize(const std::string& filename, std::shared_ptr<egt::Rect>& rect);
    void ConvertInkscapeRect2EGT(std::shared_ptr<egt::Rect>& rect);

#ifdef DO_SVG_SERIALIZATION
    void SerializeSVG(const std::string& filename, egt::SvgImage& svg, const std::string& id);
    void SerializePNG(const char* png_src, const std::string& png_dst);
#endif

private:
#ifdef DO_SVG_SERIALIZATION
    void add_svg_layer(egt::SvgImage& svg, const std::string& ss, bool is_hiding)
    {
        if (nullptr != find_layer(ss))
            return;
        auto layer = std::make_shared<egt::experimental::GaugeLayer>(svg.render(ss));
        if (is_hiding)
            layer->hide();
        add(layer);
        layer->name(ss);
        //std::cout << "render widget:" << ss << std::endl;
    }

    void add_svg_layer_with_digit(egt::SvgImage& svg, const std::string& ss, int digit, bool is_hiding)
    {
        std::ostringstream str;
        str << ss << std::to_string(digit);
        if (nullptr != find_layer(str.str()))
            return;
        auto box = svg.id_box(str.str());
        auto layer = std::make_shared<egt::experimental::GaugeLayer>(svg.render(str.str(), box));

        layer->name(str.str());
        layer->box(egt::Rect(box.x() + moat(),
                                box.y() + moat(),
                                std::ceil(box.width()),
                                std::ceil(box.height())));
        if (is_hiding)
            layer->hide();
        add(layer);
        //app_ptr->event().step();
        needle_digit += STEPPER;
        //std::cout << "render:" << str.str() << std::endl;
    }
    egt::SvgImage& m_svg;
#endif

    bool m_is_high_q_quit;
    bool m_is_low_q_quit;
    bool m_gear_deserial_finish;
    bool m_fuel_deserial_finish;
    bool m_temp_deserial_finish;
    bool m_needle_deserial_finish;
    bool m_speed_deserial_finish;
    bool m_call_deserial_finish;
    bool m_bar_deserial_finish;
    bool m_blink_deserial_finish;
    bool m_tc_deserial_finish;
    bool m_vsc_deserial_finish;
    bool m_bat_deserial_finish;
    bool m_snow_deserial_finish;
    bool m_wifi_deserial_finish;
};


class DeserializeDash
{
public:
    DeserializeDash() noexcept {}
    egt::shared_cairo_surface_t DeSerialize(const std::string& filename, std::shared_ptr<egt::Rect>& rect);
    ~DeserializeDash() {}
};

#ifdef DO_SVG_SERIALIZATION
void MotorDash::SerializeSVG(const std::string& filename, egt::SvgImage& svg, const std::string& id)
{
    auto box = svg.id_box(id);
    auto layer = std::make_shared<egt::Image>(svg.render(id, box));

    egt::detail::ErawImage e;
    const auto data = cairo_image_surface_get_data(layer->surface().get());
    const auto width = cairo_image_surface_get_width(layer->surface().get());
    const auto height = cairo_image_surface_get_height(layer->surface().get());
    e.save(solve_relative_path(filename), data, box.x(), box.y(), width, height);
}

void MotorDash::SerializePNG(const char* png_src, const std::string& png_dst)
{
    egt::shared_cairo_surface_t surface;
    egt::detail::ErawImage e;
    surface =
            egt::shared_cairo_surface_t(cairo_image_surface_create_from_png(png_src),
                                        cairo_surface_destroy);
    const auto data = cairo_image_surface_get_data(surface.get());
    const auto width = cairo_image_surface_get_width(surface.get());
    const auto height = cairo_image_surface_get_height(surface.get());
    e.save(solve_relative_path(png_dst), data, 0, 0, width, height);
}

void MotorDash::serialize_all()
{
    int i;
    std::ostringstream str;
    std::ostringstream path;

    //Serialize scale PNG image
    SerializePNG("/usr/share/egt/examples/motorcycledash/moto.png", "eraw/moto_png.eraw");

    //Serialize background image
    SerializeSVG("eraw/bkgrd.eraw", m_svg, "#bkgrd");

    //Serialize Gear
    for (i = ID_MIN_TEXT_GEAR; i <= ID_MAX_TEXT_GEAR; i += STEPPER)
    {
        str.str("");
        path.str("");
        str << "#text" << std::to_string(i);
        path << "eraw/text" << std::to_string(i) << ".eraw";
        SerializeSVG(path.str(), m_svg, str.str());
    }

    //Serialize Needles
    for (i = ID_MIN; i <= ID_MAX; i += STEPPER)
    {
        str.str("");
        path.str("");
        str << "#path" << std::to_string(i);
        path << "eraw/path" << std::to_string(i) << ".eraw";
        SerializeSVG(path.str(), m_svg, str.str());
    }

    //Serialize eng5
    SerializeSVG("eraw/eng5.eraw", m_svg, "#eng5");

    SerializeSVG("eraw/fuelrect.eraw", m_svg, "#fuelrect");

    SerializeSVG("eraw/temprect.eraw", m_svg, "#temprect");

    SerializeSVG("eraw/fuelr.eraw", m_svg, "#fuelr");

    SerializeSVG("eraw/tempw.eraw", m_svg, "#tempw");

    //Serialize tc
    SerializeSVG("eraw/tc.eraw", m_svg, "#tc");

    //Serialize mute
    SerializeSVG("eraw/mute.eraw", m_svg, "#mute");

    //Serialize takeoff
    SerializeSVG("eraw/takeoff.eraw", m_svg, "#takeoff");

    //Serialize left_blink
    SerializeSVG("eraw/left_blink.eraw", m_svg, "#left_blink");

    //Serialize right_blink
    SerializeSVG("eraw/right_blink.eraw", m_svg, "#right_blink");

    //Serialize farlight
    SerializeSVG("eraw/farlight.eraw", m_svg, "#farlight");

    //Serialize vsc
    SerializeSVG("eraw/vsc.eraw", m_svg, "#vsc");

    //Serialize wifi
    SerializeSVG("eraw/wifi.eraw", m_svg, "#wifi");

    //Serialize bt
    SerializeSVG("eraw/bt.eraw", m_svg, "#bt");

    //Serialize engine
    SerializeSVG("eraw/engine.eraw", m_svg, "#engine");

    //Serialize temp
    SerializeSVG("eraw/temp.eraw", m_svg, "#temp");

    //Serialize calling
    SerializeSVG("eraw/calling.eraw", m_svg, "#calling");

    //Serialize callname
    SerializeSVG("eraw/callname.eraw", m_svg, "#callname");

    //Serialize callnum
    SerializeSVG("eraw/callnum.eraw", m_svg, "#callnum");

    //Serialize mainspeed
    SerializeSVG("eraw/mainspeed.eraw", m_svg, "#mainspeed");

    //Serialize bat
    SerializeSVG("eraw/bat.eraw", m_svg, "#bat");

    //Serialize egoil
    SerializeSVG("eraw/egoil.eraw", m_svg, "#egoil");

    //Serialize hazards
    SerializeSVG("eraw/hazards.eraw", m_svg, "#hazards");

    //Serialize snow
    SerializeSVG("eraw/snow.eraw", m_svg, "#snow");

    //Serialize abs
    SerializeSVG("eraw/abs.eraw", m_svg, "#abs");

    //Serialize lspeed
    SerializeSVG("eraw/lspeed.eraw", m_svg, "#lspeed");

    //Serialize rspeed
    SerializeSVG("eraw/rspeed.eraw", m_svg, "#rspeed");

    //Serialize lbar
    SerializeSVG("eraw/lbar.eraw", m_svg, "#lbar");

    //Serialize rbar
    SerializeSVG("eraw/rbar.eraw", m_svg, "#rbar");

    //Create a finish indicator
    if (-1 == system("touch /root/serialize_done"))
    {
        std::cout << "touch /root/serialize_done failed, please check permission!!!" << std::endl;
        return;
    }
    if (-1 == system("sync"))
    {
        std::cout << "sync failed, please check permission!!!" << std::endl;
        return;
    }
}
#endif

egt::shared_cairo_surface_t DeserializeDash::DeSerialize(const std::string& filename, std::shared_ptr<egt::Rect>& rect)
{
    return egt::detail::ErawImage::load(filename, rect);
}

egt::shared_cairo_surface_t MotorDash::DeSerialize(const std::string& filename, std::shared_ptr<egt::Rect>& rect)
{
    return egt::detail::ErawImage::load(solve_relative_path(filename), rect);
}

//When drawing in Inkscape, the y is starting from bottom but not from top,
//so we need convert this coordinate to EGT using, the y is starting from top
void MotorDash::ConvertInkscapeRect2EGT(std::shared_ptr<egt::Rect>& rect)
{
    rect->y(screen()->size().height() - rect->y() - rect->height());
}

//using QueueCallback = std::function<void ()>;
using QueueCallback = std::function<std::string ()>;

int main(int argc, char** argv)
{
    std::cout << "EGT start" << std::endl;
    //int timediff = 0;
    //struct timeval time1, time2;
    //gettimeofday(&time1, NULL);

    std::queue<QueueCallback> high_pri_q;
    std::queue<QueueCallback> low_pri_q;

#ifdef CAN_DEBUG
    //Communication by CAN bus
    struct sockaddr_can addr;
    int sockfd;
    int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;
    struct pollfd pollfds;
    ssize_t n;
    struct can_frame frame;
#endif

    //Widget handler
    std::vector<std::shared_ptr<egt::experimental::GaugeLayer>> NeedleBase;
    std::vector<std::shared_ptr<egt::experimental::GaugeLayer>> GearBase;
    std::shared_ptr<egt::experimental::GaugeLayer> TempPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> FuelPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> TempwPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> FuelrPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> callingPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> mutePtr;
    std::shared_ptr<egt::experimental::GaugeLayer> takeoffPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> left_blinkPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> right_blinkPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> farlightPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> vscPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> wifiPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> btPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> enginePtr;
    std::shared_ptr<egt::experimental::GaugeLayer> batPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> egoilPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> hazardsPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> snowPtr;
    std::shared_ptr<egt::experimental::GaugeLayer> absPtr;

    auto rect = std::make_shared<egt::Rect>();
    std::ostringstream str;
    std::ostringstream path;

#ifdef DO_SVG_SERIALIZATION
    bool need_serialization = false;
#endif

#ifdef HAVE_LIBPLANES
    //Scale effect variables
    float scale_factor = 0.01;
    bool is_scale_rev = false;
    bool is_scale_2_max = false;
    bool is_scale_finish = false;
#endif

    egt::Application app(argc, argv);  //This call will cost ~270ms on 9x60ek board
    egt::TopWindow window;
    window.color(egt::Palette::ColorId::bg, egt::Palette::black);

#ifdef EXAMPLEDATA
    egt::add_search_path(EXAMPLEDATA);
#endif

#ifdef DO_SVG_SERIALIZATION
    //Check if serialize indicator "/serialize_done" exist? If not, need serialization
    if (access("/root/serialize_done", F_OK))
    {
        if (-1 == system("rm -rf /root/eraw"))
        {
            std::cout << "rm -rf /root/eraw failed, please check permission!!!" << std::endl;
            return -1;
        }
        if (0 > mkdir("/root/eraw", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
        {
            std::cout << "Create serialization dir eraw failed, please check permission!!!" << std::endl;
            return -1;
        }
        else
            need_serialization = true;
    }

    //If need serialization, use serialize.svg to parse SVG parameters, else use a blank one
    std::string svgpath = need_serialization ? "file:serialize.svg" : "file:deserialize.svg";
    auto svg = std::make_unique<egt::SvgImage>(svgpath, egt::SizeF(window.content_area().width(), 0));
    MotorDash motordash(*svg);
#else
    MotorDash motordash;
#endif

    window.add(motordash);
    motordash.show();
    window.show();
    motordash.add_text_widget("#textinit", "0", egt::Rect(900, 600, 2, 2), 1);
    std::cout << "EGT show" << std::endl;

    //Lambda for de-serializing background and needles
    auto DeserialNeedles = [&]()
    {
        //Background image and needles should be de-serialized firstly before main() return
        motordash.add(std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/bkgrd.eraw", rect))));
        for (auto i = ID_MIN, j =0; i <= ID_MAX; i += STEPPER, j++)
        {
            path.str("");
            path << "eraw/path" << std::to_string(i) << ".eraw";
            NeedleBase.push_back(std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize(path.str(), rect))));
            NeedleBase[j]->box(*rect);
            NeedleBase[j]->hide();
            motordash.add(NeedleBase[j]);
        }
        motordash.add_text_widget("#speed", "0", egt::Rect(298, 145, 200, 120), 180);
    };

#ifdef DO_SVG_SERIALIZATION
    if (need_serialization)
    {
        //If need serialization, show indicator for user on screen
        auto text = std::make_shared<egt::TextBox>("EGT is serializing, please wait...", egt::Rect(70, 190, 700, 200));
        text->border(0);
        text->font(egt::Font(50, egt::Font::Weight::normal));
        text->color(egt::Palette::ColorId::bg, egt::Palette::transparent);
        text->color(egt::Palette::ColorId::text, egt::Palette::red);
        window.add(text);
        app.event().step();
        motordash.serialize_all();
        text->clear();
        text->text("Serialize successfully, welcome!");
        app.event().step();
        sleep(1);
        text->hide();
    }
#endif



#ifdef HAVE_LIBPLANES
    std::cout << "Have libplanes" << std::endl;
    //Create scale image to HEO overlay
    egt::Sprite scale_s(egt::Image(motordash.DeSerialize("eraw/moto_png.eraw", rect)), egt::Size(400, 240), 1, egt::Point(200, 120),
                           egt::PixelFormat::xrgb8888, egt::WindowHint::heo_overlay);
    egt::detail::KMSOverlay* scale_ovl = reinterpret_cast<egt::detail::KMSOverlay*>(scale_s.screen());
    plane_set_pan_size(scale_ovl->s(), 0, 0);
    motordash.add(scale_s);
    scale_s.show();
#endif



#ifdef EGT_STATISTICS_ENABLE
    std::default_random_engine e(time(0));
    std::uniform_int_distribution<unsigned> randi(ID_MIN, ID_MAX);
    int randnum = 0;
    int randnum2 = 0;
    int loop = 0;
    int start = 0;
    int end = 0;
#else
    int gear_index = 0;
#ifdef CAN_DEBUG
    int last_needle_index = 0;
#else
    int speed_index = 0;
    int needle_index = 0;
#endif
    bool is_needle_finish = false;
    std::string lbd_ret = "0";
    //bool is_first_time_run_needle = true;
    //int bklt = 7;  //max backlight brightness
    //bool is_backlight_rev = false;
    //int render_times = 0;
    int timer_cnt;
    int lbar = 7;
    int temp_index = 0;
    int fuel_index = 0;
#endif

#ifdef CAN_DEBUG
    //Init socket
    if ((sockfd = socket(family, type, proto)) < 0)
    {
        perror("socket");
        return 1;
    }

    addr.can_family = family;
    addr.can_ifindex = 0;

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        close(sockfd);
        return 1;
    }

    pollfds.fd = sockfd;
    pollfds.events = POLLRDNORM;

    auto dbg_print_frame = [](struct can_frame *frame)
    {
        printf("%04x: ", frame->can_id);
        if (frame->can_id & CAN_RTR_FLAG)
       	{
            printf("remote request");
        }
       	else
       	{
            printf("[%d]", frame->can_dlc);
            for (int i = 0; i < frame->can_dlc; i++)
                printf(" %02x", frame->data[i]);
        }
        printf("\n");
    };
#endif

#if 1 //use timer or screen click to debug? 1:use timer; 0:use click
    egt::PeriodicTimer timer(std::chrono::milliseconds(10));
    timer.on_timeout([&]()
    {
#ifdef CAN_DEBUG
        if (0 < poll(&pollfds, 1, 0))
        {
            if ((n = read(sockfd , &frame, sizeof(can_frame))) < 0)
            {
                if (errno == ECONNRESET)
                {
                    perror("errorno ECONNRESET");
                }
                else
                {
                    perror("error when server read from client");
                }
            }
            else if (n == 0)
            {
                perror("socket read 0");
            }
            else
            {

                dbg_print_frame(&frame);
                frame.data[0] = (0x63 < frame.data[0]) ? frame.data[0]%0x63 : frame.data[0];  //main speed digit
                frame.data[1] = (120 < frame.data[1]) ? frame.data[1]%120 : frame.data[1];    //rotating needles
                frame.data[2] = (90 < frame.data[2]) ? frame.data[2]%90 : frame.data[2];      //temperature digit

            }
        }
#endif

        //motordash.get_high_q_state/get_low_q_state are used to protect high q not be interrupted by other event re-enter
        //if the state is false, it means this queue has not execute finished yet.
        if (!motordash.get_high_q_state())
            return;
        if (!high_pri_q.empty())
        {
            //gettimeofday(&time1, NULL);
            motordash.set_high_q_state(false);
            lbd_ret = high_pri_q.front()();
            high_pri_q.pop();
            motordash.set_high_q_state(true);
            //gettimeofday(&time2, NULL);
            //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
            //if (HIGH_Q_TIME_THRESHHOLD <= timediff)
                //std::cout << "warning!!! high pri q: " << lbd_ret <<"() exec too longer: " << timediff << "us" << std::endl;

        }
        else
        {
            if (!motordash.get_low_q_state())
                return;
            if (!low_pri_q.empty())
            {
                //gettimeofday(&time1, NULL);
                motordash.set_low_q_state(false);
                lbd_ret = low_pri_q.front()();
                low_pri_q.pop();
                motordash.set_low_q_state(true);
                //gettimeofday(&time2, NULL);
                //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                //if (LOW_Q_TIME_THRESHHOLD <= timediff)
                    //std::cout << "warning!!! low pri q: " << lbd_ret <<"() exec too longer: " << timediff << "us" << std::endl;
                return;
            }
        }

#ifdef EGT_STATISTICS_ENABLE
        motordash.hide_all();
        randnum = randi(e);
        while ((randnum - ID_MIN)%4)
            randnum = randi(e);
        randnum2 = randi(e);
        while ((randnum2 - ID_MIN)%4)
            randnum2 = randi(e);
        if (randnum > randnum2)
        {
            loop = (randnum - randnum2)/4 + 1;
            start = randnum2;
            end = randnum;
            std::cout << start << "," << end << ",";
        }
        else
        {
            loop = (randnum2 - randnum)/4 + 1;
            start = randnum;
            end = randnum2;
            std::cout << start << "," << end << ",";
        }

        g_digit = start;
        is_increasing = true;

        gettimeofday(&time1, NULL);
        for (int i=0; i < loop; i++)
        {
            motordash.show_digit(g_digit);
            g_digit += STEPPER;
        }
        gettimeofday(&time2, NULL);
        timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
        std::cout << timediff << std::endl;
#else
        //viariable to control the animation and frequency
        timer_cnt = (LOW_Q_TIME_THRESHHOLD <= timer_cnt) ? 0 : timer_cnt + 1;

        //needle move function implemented by lambda
        auto needle_move = [&]()
        {
#ifdef CAN_DEBUG
            int i;
            if (last_needle_index < frame.data[1])  //increase
            {
                for (i = last_needle_index + 1; i <= frame.data[1]; i++)
                {
                    NeedleBase[i]->show();
                }
            }
            else if (last_needle_index > frame.data[1])  //decrease
            {
                for (i = last_needle_index; i > frame.data[1]; i--)
                {
                    NeedleBase[i]->hide();
                }
            }

            last_needle_index = frame.data[1];
            is_needle_finish = true;
#else
            //needles
            for (int i=0; i<8; i++)
            {
                if (is_increasing)
                {
                    NeedleBase[needle_index]->show();
                }
                else
                {
                    if (!(timer_cnt % 9))
                    {
                        NeedleBase[needle_index]->hide();
                    }
                    else
                    {
                        return;
                    }
                }

                if (is_increasing && MAX_NEEDLE_INDEX == needle_index)
                {
                    is_increasing = false;
                    is_needle_finish = true;
                    return "needle_move";
                }
                else if (!is_increasing && 0 == needle_index)
                {
                    is_increasing = true;
                    is_needle_finish = true;
                    return "needle_move";
                }
                else
                {
                    needle_index = is_increasing ? needle_index + 1 : needle_index - 1;
                }
            }
#endif
            return "needle_move";
        };

#if 1
        //make sure the needles run a circle finish, then can enter to low priority procedure
        if (is_needle_finish)
        {
            is_needle_finish = false;

            //deserialize speed
            if (!motordash.get_speed_deserial_state())
            {
                //low_pri_q.push([&]()
                //{
                    //gettimeofday(&time1, NULL);
                    //De-serialize main_speed, get the main speed's position, time cost almostly 110ms to de-serialize main speed
                    //auto mainspeed = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("/eraw/mainspeed.eraw", rect)));
                    //motordash.ConvertInkscapeRect2EGT(rect);
                    //motordash.add_text_widget("#speed", "0", *rect, 180);  //Inkscape's rect has gap with reality
                    //motordash.add_text_widget("#speed", "0", egt::Rect(298, 145, 200, 120), 180);
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Create text mainspeed time: " << timediff << " us" << std::endl;
                    //return "mainspeed_text";
                //});
                low_pri_q.push([&]()
                {
                    //gettimeofday(&time1, NULL);
                    //De-serialize lspeed, get the main speed's position, time cost almostly 110ms to de-serialize main speed
                    //auto lspeed = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("/eraw/lspeed.eraw", rect)));
                    //motordash.ConvertInkscapeRect2EGT(rect);
                    //motordash.add_text_widget("#lspeed", "0", *rect, 18);
                    motordash.add_text_widget("#lspeed", "0", egt::Rect(165, 448, 20, 20), 18);
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Create text lspeed time: " << timediff << " us" << std::endl;
                    return "lspeed_text";
                });
                low_pri_q.push([&]()
                {
                    //gettimeofday(&time1, NULL);
                    //De-serialize rspeed, get the main speed's position, time cost almostly 110ms to de-serialize main speed
                    //auto rspeed = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("/eraw/rspeed.eraw", rect)));
                    //motordash.ConvertInkscapeRect2EGT(rect);
                    //motordash.add_text_widget("#rspeed", "0", *rect, 18);
                    motordash.add_text_widget("#rspeed", "0", egt::Rect(710, 443, 20, 20), 18);
                    motordash.add_text_widget("#tmp", "0", egt::Rect(745, 96, 20, 20), 18);
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Create text rspeed time: " << timediff << " us" << std::endl;
                    return "rspeed_text";
                });
                motordash.set_speed_deserial_state(true);
                return;
            }
            else
            {
#ifdef CAN_DEBUG
                low_pri_q.push([&]()
                {
                    motordash.find_text("#speed")->text(std::to_string(frame.data[0]));
                    motordash.find_text("#lspeed")->text(std::to_string(frame.data[0]));
                    motordash.find_text("#rspeed")->text(std::to_string(frame.data[0]));
                    motordash.find_text("#tmp")->text(std::to_string(frame.data[2]));
                    return "speed_change";
                });
#else
                if (!(timer_cnt % 1))
                {
                    low_pri_q.push([&]()
                    {
                        speed_index = (5 <= speed_index) ? 0 : speed_index + 1;
                        motordash.find_text("#speed")->clear();
                        motordash.find_text("#lspeed")->clear();
                        motordash.find_text("#rspeed")->clear();
                        motordash.find_text("#tmp")->clear();
                        switch (speed_index)
                        {
                            case 0:
                                motordash.find_text("#speed")->text("15");
                                motordash.find_text("#lspeed")->text("15");
                                motordash.find_text("#rspeed")->text("15");
                                motordash.find_text("#tmp")->text("30");
                                break;
                            case 1:
                                motordash.find_text("#speed")->text("28");
                                motordash.find_text("#lspeed")->text("28");
                                motordash.find_text("#rspeed")->text("28");
                                motordash.find_text("#tmp")->text("35");
                                break;
                            case 2:
                                motordash.find_text("#speed")->text("40");
                                motordash.find_text("#lspeed")->text("40");
                                motordash.find_text("#rspeed")->text("40");
                                motordash.find_text("#tmp")->text("37");
                                break;
                            case 3:
                                motordash.find_text("#speed")->text("69");
                                motordash.find_text("#lspeed")->text("69");
                                motordash.find_text("#rspeed")->text("69");
                                motordash.find_text("#tmp")->text("42");
                                break;
                            case 4:
                                //motordash.find_text("#speed")->color(egt::Palette::ColorId::text, egt::Palette::red);
                                motordash.find_text("#speed")->text("88");
                                motordash.find_text("#lspeed")->text("88");
                                motordash.find_text("#rspeed")->text("88");
                                motordash.find_text("#tmp")->text("55");
                                break;
                            default:
                                motordash.find_text("#speed")->text("0");
                                motordash.find_text("#lspeed")->text("0");
                                motordash.find_text("#rspeed")->text("0");
                                motordash.find_text("#tmp")->text("30");
                                break;
                        }

                        return "speed_change";
                    });
                }
#endif
            }

            //this branch exec the gear de-serialize only execute once
            if (!motordash.get_gear_deserial_state())
            {
                low_pri_q.push([&]()
                {
                    //gettimeofday(&time1, NULL);
                    //De-serialize gear
                    for (int i = ID_MIN_TEXT_GEAR, j =0; i <= ID_MAX_TEXT_GEAR; i += STEPPER, j++)
                    {
                        path.str("");
                        path << "eraw/text" << std::to_string(i) << ".eraw";
                        GearBase.push_back(std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize(path.str(), rect))));
                        GearBase[j]->box(*rect);
                        GearBase[j]->hide();
                        motordash.add(GearBase[j]);
                    }
                    GearBase[0]->show();  //initialize gear N
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Derialize gear time: " << timediff << " us" << std::endl;
                    return "gear_deserial";
                });
                motordash.set_gear_deserial_state(true);
                return;
#if 0
                //low_pri_q.push([&](){motordash.add_rec_widget_with_text("#test", "test");});
                //low_pri_q.push([&](){motordash.render_bar(); return "render_bar";});
                //low_pri_q.push([&](){motordash.render_bar5(); return "render_bar5";});
                low_pri_q.push([&](){motordash.render_bar8(); return "render_bar8";});
                //low_pri_q.push([&](){motordash.render_gear(); return "render_gear";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#calling", egt::Rect(291, 150, 35, 36), true); return "render_calling";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#left_blink", egt::Rect(138, 10, 48, 33), false); return "render_left_blink";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#right_blink", egt::Rect(632, 10, 48, 32.5), false); return "render_right_blink";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#vsc", egt::Rect(229, 16, 50, 20), false); return "render_vsc";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#farlight", egt::Rect(193.3, 15, 34, 22.2), false); return "render_farlight";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#takeoff", egt::Rect(132, 59, 49, 38.4), false); return "render_takeoff";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#mute", egt::Rect(62, 108, 30, 30), false); return "render_mute";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#bat", egt::Rect(61, 416, 40, 25), false); return "render_bat";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#egoil", egt::Rect(106, 416, 62, 25), false); return "render_egoil";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#hazards", egt::Rect(228, 387, 43, 41), false); return "render_hazards";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#snow", egt::Rect(485, 386, 45, 45), false); return "render_snow";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#abs", egt::Rect(532, 388, 60, 40), false); return "render_abs";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#engine", egt::Rect(618, 54, 66, 38), false); return "render_engine";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#bt", egt::Rect(588, 11, 29, 37), false); return "render_bt";});
                low_pri_q.push([&](){motordash.add_rec_widget_with_rec("#wifi", egt::Rect(545, 6, 40, 40), false); return "render_wifi";});

                low_pri_q.push([&](){motordash.add_text_widget("#tc", "3", egt::Rect(67, 51, 20, 20), 30); return "render_tc";});
                low_pri_q.push([&](){motordash.add_text_widget("#lkm", "55", egt::Rect(165, 448, 20, 20), 18); return "render_lkm";});
                low_pri_q.push([&](){motordash.add_text_widget("#rkm", "55", egt::Rect(710, 443, 20, 20), 18); return "render_rkm";});
                low_pri_q.push([&](){motordash.add_text_widget("#tmp", "", egt::Rect(705, 96, 20, 20), 18); return "render_tmp";});
                low_pri_q.push([&](){motordash.add_text_widget("#callname", "EGT", egt::Rect(335, 120, 80, 40), 30); return "render_callname";});
                low_pri_q.push([&](){motordash.add_text_widget("#phone", "13883052865", egt::Rect(335, 155, 100, 22), 21); return "render_phone";});
                low_pri_q.push([&](){motordash.add_text_widget("#lbar", "0.5", egt::Rect(240, 431, 20, 20), 20, egt::Palette::red); return "render_lbar";});
                low_pri_q.push([&](){motordash.add_text_widget("#rbar", "0.8", egt::Rect(496, 431, 20, 20), 20, egt::Palette::red); return "render_rbar";});
                low_pri_q.push([&](){motordash.add_text_widget("#gearnum", "N", egt::Rect(713, -13, 80, 80), 80); return "render_gearnum";});
                low_pri_q.push([&](){motordash.add_text_widget("#speed", "0", egt::Rect(298, 145, 200, 120), 180); return "render_speed";});
                low_pri_q.push([&](){motordash.add_text_widget("#engine5", "5", egt::Rect(82, 1, 20, 2), 40); return "render_engine5";});
                //low_pri_q.push([&](){motordash.render_temp_l();});
                //low_pri_q.push([&](){motordash.render_temp_r();});
                //low_pri_q.push([&](){motordash.test();});

                low_pri_q.push([&]()
                {
                    auto text = std::make_shared<egt::TextBox>("5", egt::Rect(egt::Point(82, 1), egt::Size(20, 2)));
                    text->border(0);
                    text->font(egt::Font(40, egt::Font::Weight::normal));
                    text->color(egt::Palette::ColorId::bg, egt::Palette::transparent);
                    text->color(egt::Palette::ColorId::text, egt::Palette::white);
                    window.add(text);
                });
#endif
            }
            else
            {
#if 0
                if (!(timer_cnt % 7))
                {
                    low_pri_q.push([&]()
                    {
                        motordash.hide_gear_text(GearBase);
                        gear_index = (5 <= gear_index) ? 0 : gear_index;
                        GearBase[gear_index++]->show();
                        return "gear";
                    });
                }
#endif
            }

            //deserialize temp
            if (!motordash.get_temp_deserial_state())
            {
                low_pri_q.push([&]()
                {
                    TempwPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/tempw.eraw", rect)));
                    TempwPtr->box(*rect);
                    TempwPtr->hide();
                    motordash.add(TempwPtr);

                    TempPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/temprect.eraw", rect)));
                    TempPtr->box(*rect);
                    TempPtr->hide();
                    motordash.add(TempPtr);
                    return "temp_deserial6";
                });
                motordash.set_temp_deserial_state(true);
                return;
            }
            else
            {
                if (!(timer_cnt % 5))
                {
                    low_pri_q.push([&]()
                    {
                        if (7 <= temp_index)
                            temp_index = 0;
                        switch (temp_index)
                        {
                            case 0:
                                TempPtr->hide();
                                TempwPtr->show();
                                TempwPtr->y(g_temp_table[0].y);
                                TempwPtr->height(g_temp_table[0].h);
                                break;
                            case 1:
                                TempPtr->hide();
                                TempwPtr->show();
                                TempwPtr->y(g_temp_table[1].y);
                                TempwPtr->height(g_temp_table[1].h);
                                break;
                            case 2:
                                TempPtr->hide();
                                TempwPtr->show();
                                TempwPtr->y(g_temp_table[2].y);
                                TempwPtr->height(g_temp_table[2].h);
                                break;
                            case 3:
                                TempPtr->hide();
                                TempwPtr->show();
                                TempwPtr->y(g_temp_table[3].y);
                                TempwPtr->height(g_temp_table[3].h);
                                break;
                            case 4:
                                TempPtr->hide();
                                TempwPtr->show();
                                TempwPtr->y(g_temp_table[4].y);
                                TempwPtr->height(g_temp_table[4].h);
                                break;
                            case 5:
                                TempPtr->hide();
                                TempwPtr->show();
                                TempwPtr->y(g_temp_table[5].y);
                                TempwPtr->height(g_temp_table[5].h);
                                break;
                            case 6:
                                TempwPtr->hide();
                                TempPtr->show();
                                break;
                            default:
                                std::cout << "temp full!" << std::endl;
                                break;
                        }
                        temp_index++;
                        return "temp";
                    });
                }
            }

            //deserialize fuel
            if (!motordash.get_fuel_deserial_state())
            {
                low_pri_q.push([&]()
                {
                    FuelrPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/fuelr.eraw", rect)));
                    FuelrPtr->box(*rect);
                    FuelrPtr->hide();
                    motordash.add(FuelrPtr);
                    FuelPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/fuelrect.eraw", rect)));
                    FuelPtr->box(*rect);
                    FuelPtr->show();
                    motordash.add(FuelPtr);
                    return "fuel_deserial";
                });
                motordash.set_fuel_deserial_state(true);
                return;
            }
            else
            {
                if (!(timer_cnt % 3))
                {
                    low_pri_q.push([&]()
                    {
                        if (8 <= fuel_index)
                            fuel_index = 0;
                        switch (fuel_index)
                        {
                            case 0:
                                FuelrPtr->hide();
                                FuelPtr->show();
                                FuelPtr->y(g_fuel_table[0].y);
                                FuelPtr->height(g_fuel_table[0].h);
                                break;
                            case 1:
                                FuelrPtr->hide();
                                FuelPtr->show();
                                FuelPtr->y(g_fuel_table[1].y);
                                FuelPtr->height(g_fuel_table[1].h);
                                break;
                            case 2:
                                FuelrPtr->hide();
                                FuelPtr->show();
                                FuelPtr->y(g_fuel_table[2].y);
                                FuelPtr->height(g_fuel_table[2].h);
                                break;
                            case 3:
                                FuelrPtr->hide();
                                FuelPtr->show();
                                FuelPtr->y(g_fuel_table[3].y);
                                FuelPtr->height(g_fuel_table[3].h);
                                break;
                            case 4:
                                FuelrPtr->hide();
                                FuelPtr->show();
                                FuelPtr->y(g_fuel_table[4].y);
                                FuelPtr->height(g_fuel_table[4].h);
                                break;
                            case 5:
                                FuelrPtr->hide();
                                FuelPtr->show();
                                FuelPtr->y(g_fuel_table[5].y);
                                FuelPtr->height(g_fuel_table[5].h);
                                break;
                            case 6:
                                FuelrPtr->hide();
                                FuelPtr->show();
                                FuelPtr->y(g_fuel_table[6].y);
                                FuelPtr->height(g_fuel_table[6].h);
                                break;
                            case 7:
                                FuelPtr->hide();
                                FuelrPtr->show();
                                break;
                            default:
                                std::cout << "fuel full!" << std::endl;
                                break;
                        }
                        fuel_index++;
                        return "fuel";
                    });
                }
            }



            //deserialize call
            if (!motordash.get_call_deserial_state())
            {
                low_pri_q.push([&]()
                {
                    motordash.add_text_widget("#callname", "EGT", egt::Rect(335, 120, 80, 40), 30);
                    return "callname_text";
                });
                low_pri_q.push([&]()
                {
                    motordash.add_text_widget("#phone", "13883052865", egt::Rect(335, 155, 100, 22), 21);
                    return "phone_text";
                });
                low_pri_q.push([&]()
                {
                    //gettimeofday(&time1, NULL);
                    //De-serialize call
                    callingPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/calling.eraw", rect)));
                    callingPtr->box(*rect);
                    callingPtr->hide();
                    motordash.add(callingPtr);
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Create text call time: " << timediff << " us" << std::endl;
                    return "call_text";
                });
                motordash.set_call_deserial_state(true);
                return;
            }
            else
            {

            }

            //deserialize eng5 & tc
            if (!motordash.get_tc_deserial_state())
            {
                low_pri_q.push([&]()
                {
                    //gettimeofday(&time1, NULL);
                    //De-serialize eng5 & tc
                    low_pri_q.push([&]()
                    {
                        motordash.add_text_widget("#engine5", "5", egt::Rect(82, 1, 20, 2), 40);
                        return "callname_text";
                    });
                    low_pri_q.push([&]()
                    {
                        motordash.add_text_widget("#tc", "3", egt::Rect(67, 51, 20, 20), 30);
                        return "phone_text";
                    });
                    mutePtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/mute.eraw", rect)));
                    mutePtr->box(*rect);
                    motordash.add(mutePtr);
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Create text eng5 & tc time: " << timediff << " us" << std::endl;
                    return "eng5tc_text";
                });
                motordash.set_tc_deserial_state(true);
                return;
            }
            else
            {

            }

            //deserialize bar
            if (!motordash.get_bar_deserial_state())
            {
                low_pri_q.push([&]()
                {
                    //gettimeofday(&time1, NULL);
                    //De-serialize tmp
                    motordash.add_text_widget("#lbar", "0.5", egt::Rect(240, 431, 20, 20), 20, egt::Palette::red);
                    motordash.add_text_widget("#rbar", "0.8", egt::Rect(496, 431, 20, 20), 20, egt::Palette::red);
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Create text bar time: " << timediff << " us" << std::endl;
                    return "bar_text";
                });
                motordash.set_bar_deserial_state(true);
                return;
            }
            else
            {
                if (!(timer_cnt % 11))
                {
                    low_pri_q.push([&]()
                    {
                        //bar
                        motordash.find_text("#lbar")->clear();
                        motordash.find_text("#rbar")->clear();
                        if (7 == lbar)
                        {
                            motordash.find_text("#lbar")->text("0.3");
                            motordash.find_text("#rbar")->text("0.6");
                            lbar = 3;
                        }
                        else
                        {
                            motordash.find_text("#lbar")->text("0.7");
                            motordash.find_text("#rbar")->text("0.9");
                            lbar = 7;
                        }
                        return "bar_change";
                    });
                }
            }

            //deserialize blink
            if (!motordash.get_blink_deserial_state())
            {
                low_pri_q.push([&]()
                {
                    //gettimeofday(&time1, NULL);
                    left_blinkPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/left_blink.eraw", rect)));
                    left_blinkPtr->box(*rect);
                    motordash.add(left_blinkPtr);
                    right_blinkPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/right_blink.eraw", rect)));
                    right_blinkPtr->box(*rect);
                    motordash.add(right_blinkPtr);
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Desrial blink time: " << timediff << " us" << std::endl;
                    return "blink_deserial";
                });
                motordash.set_blink_deserial_state(true);
                return;
            }
            else
            {
                if (!(timer_cnt % 3))
                {
                    low_pri_q.push([&]()
                    {
                        //blink
                        if (left_blinkPtr->visible())
                            left_blinkPtr->hide();
                        else
                            left_blinkPtr->show();

                        if (right_blinkPtr->visible())
                            right_blinkPtr->hide();
                        else
                            right_blinkPtr->show();

                        return "left_right_blink";
                    });
                }
            }

            //deserialize vsc & farlight & takeoff
            if (!motordash.get_vsc_deserial_state())
            {
                low_pri_q.push([&]()
                {
                    //gettimeofday(&time1, NULL);
                    vscPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/vsc.eraw", rect)));
                    vscPtr->box(*rect);
                    motordash.add(vscPtr);
                    farlightPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/farlight.eraw", rect)));
                    farlightPtr->box(*rect);
                    motordash.add(farlightPtr);
                    takeoffPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/takeoff.eraw", rect)));
                    takeoffPtr->box(*rect);
                    motordash.add(takeoffPtr);
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Desrial vsc time: " << timediff << " us" << std::endl;
                    return "vsc_deserial";
                });
                motordash.set_vsc_deserial_state(true);
                return;
            }
            else
            {
                if (!(timer_cnt % 5))
                {
                    low_pri_q.push([&]()
                    {
                        //vsc
                        if (vscPtr->visible())
                            vscPtr->hide();
                        else
                            vscPtr->show();

                        if (farlightPtr->visible())
                            farlightPtr->hide();
                        else
                            farlightPtr->show();

                        if (takeoffPtr->visible())
                            takeoffPtr->hide();
                        else
                            takeoffPtr->show();

                        return "vsc_farlight_takeoff";
                    });
                }
            }

            //deserialize bat & egoil & hazards
            if (!motordash.get_bat_deserial_state())
            {
                low_pri_q.push([&]()
                {
                    //gettimeofday(&time1, NULL);
                    batPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/bat.eraw", rect)));
                    batPtr->box(*rect);
                    motordash.add(batPtr);
                    egoilPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/egoil.eraw", rect)));
                    egoilPtr->box(*rect);
                    motordash.add(egoilPtr);
                    hazardsPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/hazards.eraw", rect)));
                    hazardsPtr->box(*rect);
                    motordash.add(hazardsPtr);
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Desrial bat time: " << timediff << " us" << std::endl;
                    return "bat_deserial";
                });
                motordash.set_bat_deserial_state(true);
                return;
            }
            else
            {
                if (!(timer_cnt % 9))
                {
                    low_pri_q.push([&]()
                    {
                        //vsc
                        if (batPtr->visible())
                            batPtr->hide();
                        else
                            batPtr->show();

                        if (egoilPtr->visible())
                            egoilPtr->hide();
                        else
                            egoilPtr->show();

                        if (hazardsPtr->visible())
                            hazardsPtr->hide();
                        else
                            hazardsPtr->show();

                        return "bat_egoil_hazards";
                    });
                }
            }

            //deserialize snow & abs & engine
            if (!motordash.get_snow_deserial_state())
            {
                low_pri_q.push([&]()
                {
                    //gettimeofday(&time1, NULL);
                    snowPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/snow.eraw", rect)));
                    snowPtr->box(*rect);
                    motordash.add(snowPtr);
                    absPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/abs.eraw", rect)));
                    absPtr->box(*rect);
                    motordash.add(absPtr);
                    enginePtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/engine.eraw", rect)));
                    enginePtr->box(*rect);
                    motordash.add(enginePtr);
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Desrial snow time: " << timediff << " us" << std::endl;
                    return "snow_deserial";
                });
                motordash.set_snow_deserial_state(true);
                return;
            }
            else
            {
                if (!(timer_cnt % 7))
                {
                    low_pri_q.push([&]()
                    {
                        //vsc
                        if (snowPtr->visible())
                            snowPtr->hide();
                        else
                            snowPtr->show();

                        if (absPtr->visible())
                            absPtr->hide();
                        else
                            absPtr->show();

                        if (enginePtr->visible())
                            enginePtr->hide();
                        else
                            enginePtr->show();

                        return "snow_abs_engin";
                    });
                }
            }

            //deserialize wifi & bt
            if (!motordash.get_wifi_deserial_state())
            {
                low_pri_q.push([&]()
                {
                    //gettimeofday(&time1, NULL);
                    wifiPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/wifi.eraw", rect)));
                    wifiPtr->box(*rect);
                    motordash.add(wifiPtr);
                    btPtr = std::make_shared<egt::experimental::GaugeLayer>(egt::Image(motordash.DeSerialize("eraw/bt.eraw", rect)));
                    btPtr->box(*rect);
                    motordash.add(btPtr);
                    //gettimeofday(&time2, NULL);
                    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                    //std::cout << "Desrial wifi time: " << timediff << " us" << std::endl;
                    return "wifi_deserial";
                });
                motordash.set_wifi_deserial_state(true);
                return;
            }
            else
            {
                if (!(timer_cnt % 9))
                {
                    low_pri_q.push([&]()
                    {
                        //vsc
                        if (wifiPtr->visible())
                            wifiPtr->hide();
                        else
                            wifiPtr->show();

                        if (btPtr->visible())
                            btPtr->hide();
                        else
                            btPtr->show();

                        return "wifi_bt";
                    });
                }
            }
#if 0
                low_pri_q.push([&]()
                {
                    //l r speed
                    //motordash.hide_speed_text();
                    //motordash.hide_left_speed_text();
                    //motordash.hide_right_speed_text();
                    //speed_table_index = (3 <= speed_table_index) ? 0 : speed_table_index;
                    //motordash.apply(g_speed_table[speed_table_index].main_speed[0], true, SvgEleType::text);
                    //motordash.apply(g_speed_table[speed_table_index].main_speed[1], true, SvgEleType::text);
                    //str.str("");
                    //str << std::to_string(g_speed_table[speed_table_index].speed_digit);
                    //motordash.find_text("#lkm")->clear();
                    //motordash.find_text("#rkm")->clear();
                    //motordash.find_text("#tmp")->clear();
                    //motordash.find_text("#lkm")->text(str.str());
                    //motordash.find_text("#rkm")->text(str.str());
                    //motordash.find_text("#tmp")->text(str.str());
                    //motordash.apply(g_speed_table[speed_table_index].left_speed[0], true, SvgEleType::text);
                    //motordash.apply(g_speed_table[speed_table_index].left_speed[1], true, SvgEleType::text);
                    //motordash.apply(g_speed_table[speed_table_index].right_speed[0], true, SvgEleType::text);
                    //motordash.apply(g_speed_table[speed_table_index].right_speed[1], true, SvgEleType::text);
                    //speed_table_index++;
                    if (5 <= temp_index)
                        temp_index = 0;
                    motordash.find_text("#lkm")->clear();
                    motordash.find_text("#rkm")->clear();
                    motordash.find_text("#tmp")->clear();
                    switch (temp_index)
                    {
                        case 0:
                            motordash.find_text("#lkm")->text("18");
                            motordash.find_text("#rkm")->text("15");
                            motordash.find_text("#tmp")->text("38");
                            break;
                        case 1:
                            motordash.find_text("#lkm")->text("33");
                            motordash.find_text("#rkm")->text("45");
                            motordash.find_text("#tmp")->text("46");
                            break;
                        case 2:
                            motordash.find_text("#lkm")->text("51");
                            motordash.find_text("#rkm")->text("66");
                            motordash.find_text("#tmp")->text("51");
                            break;
                        case 3:
                            motordash.find_text("#lkm")->text("67");
                            motordash.find_text("#rkm")->text("70");
                            motordash.find_text("#tmp")->text("56");
                            break;
                        case 4:
                            motordash.find_text("#lkm")->text("73");
                            motordash.find_text("#rkm")->text("88");
                            motordash.find_text("#tmp")->text("61");
                            break;
                        default:
                            motordash.find_text("#lkm")->text("18");
                            motordash.find_text("#rkm")->text("15");
                            motordash.find_text("#tmp")->text("38");
                            break;
                    }
                    temp_index++;
                    return "speed_temp_change";
                });

                low_pri_q.push([&]()
                {
                    //bar
                    motordash.find_text("#lbar")->clear();
                    motordash.find_text("#rbar")->clear();
                    if (even_odd)
                    {
                        motordash.find_text("#lbar")->text("0.7");
                        motordash.find_text("#rbar")->text("0.6");
                    }
                    else
                    {
                        motordash.find_text("#lbar")->text("0.3");
                        motordash.find_text("#rbar")->text("0.9");
                    }
                    return "bar_change";
                });

                low_pri_q.push([&]()
                {
                    //blink
                    if (even_odd)
                    {
                        motordash.find_rec("#left_blink")->hide();
                        motordash.find_rec("#right_blink")->show();
                    }
                    else
                    {
                        motordash.find_rec("#left_blink")->show();
                        motordash.find_rec("#right_blink")->hide();
                    }
                    return "left_right_blink";
                });

                low_pri_q.push([&]()
                {
                    //wifi
                    if (even_odd)
                    {
                        motordash.find_rec("#bt")->show();
                        motordash.find_rec("#wifi")->hide();
                    }
                    else
                    {
                        motordash.find_rec("#bt")->hide();
                        motordash.find_rec("#wifi")->show();
                    }
                    return "bt_wifi_blink";
                });

                low_pri_q.push([&]()
                {
                    //engine
                    if (even_odd)
                    {
                        motordash.find_rec("#engine")->hide();
                        motordash.find_rec("#abs")->show();
                    }
                    else
                    {
                        motordash.find_rec("#engine")->show();
                        motordash.find_rec("#abs")->hide();
                    }
                    return "engine_abs_blink";
                });

                low_pri_q.push([&]()
                {
                    //bat
                    if (even_odd)
                    {
                        motordash.find_rec("#bat")->hide();
                        motordash.find_rec("#egoil")->show();
                    }
                    else
                    {
                        motordash.find_rec("#bat")->show();
                        motordash.find_rec("#egoil")->hide();
                    }
                    return "bat_egoil_blink";
                });

                low_pri_q.push([&]()
                {
                    //snow
                    if (even_odd)
                    {
                        motordash.find_rec("#snow")->hide();
                        motordash.find_rec("#hazards")->show();
                    }
                    else
                    {
                        motordash.find_rec("#snow")->show();
                        motordash.find_rec("#hazards")->hide();
                    }
                    return "snow_hazards_blink";
                });

                low_pri_q.push([&]()
                {
                    //mute
                    if (even_odd)
                    {
                        motordash.find_rec("#mute")->hide();
                        motordash.find_rec("#farlight")->show();
                    }
                    else
                    {
                        motordash.find_rec("#mute")->show();
                        motordash.find_rec("#farlight")->hide();
                    }
                    return "mute_farlight_blink";
                });

                low_pri_q.push([&]()
                {
                    //vsc
                    if (even_odd)
                    {
                        motordash.find_rec("#vsc")->hide();
                        motordash.find_rec("#takeoff")->show();
                    }
                    else
                    {
                        motordash.find_rec("#vsc")->show();
                        motordash.find_rec("#takeoff")->hide();
                    }
                    return "mute_takeoff_blink";
                });

                if (motordash.get_animation_state())
                    low_pri_q.push(show_fuel);

                low_pri_q.push([&]()
                {
                    //gear/speed
                    if (5 <= gear_index)
                        gear_index = 0;
                    motordash.find_text("#gearnum")->color(egt::Palette::ColorId::text, egt::Palette::white);
                    //motordash.find_text("#speed")->color(egt::Palette::ColorId::text, egt::Palette::white);
                    motordash.find_text("#gearnum")->clear();
                    motordash.find_text("#speed")->clear();
                    switch (gear_index)
                    {
                        case 0:
                            motordash.find_text("#gearnum")->text("1");
                            motordash.find_text("#speed")->text("15");
                            break;
                        case 1:
                            motordash.find_text("#gearnum")->text("2");
                            motordash.find_text("#speed")->text("28");
                            break;
                        case 2:
                            motordash.find_text("#gearnum")->text("3");
                            motordash.find_text("#speed")->text("40");
                            break;
                        case 3:
                            motordash.find_text("#gearnum")->text("4");
                            motordash.find_text("#speed")->text("69");
                            break;
                        case 4:
                            //motordash.find_text("#speed")->color(egt::Palette::ColorId::text, egt::Palette::red);
                            motordash.find_text("#speed")->text("88");
                            motordash.find_text("#gearnum")->color(egt::Palette::ColorId::text, egt::Palette::green);
                            motordash.find_text("#gearnum")->text("N");
                            break;
                        default:
                            motordash.find_text("#gearnum")->text("1");
                            break;
                    }
                    gear_index++;

                    if (!motordash.get_animation_state())
                    {
                        motordash.set_animation_state(true);
                        //std::cout << "start-up ready, animation can be activated" << std::endl;
                    }

                    return "gear_speed_change";
                });
#if 0
                low_pri_q.push([&]()
                {
                    //temperature digit
                    motordash.hide_temp_text();
                    temp_table_index = (MAX_TEMP_TABLE <= temp_table_index) ? 0 : temp_table_index;
                    motordash.apply(g_temp_table[temp_table_index][0], true, SvgEleType::text);
                    motordash.apply(g_temp_table[temp_table_index][1], true, SvgEleType::text);
                    temp_table_index++;
                });


                low_pri_q.push([&]()
                {
                    //gear
                    motordash.hide_gear_text();
                    motordash.apply(g_gear_txt_digit, true, SvgEleType::text);
                    if (is_gear_inc && ID_MAX_TEXT_GEAR == g_gear_txt_digit)
                    {
                        is_gear_inc = false;
                    }
                    else if (!is_gear_inc && ID_MIN_TEXT_GEAR == g_gear_txt_digit)
                    {
                        is_gear_inc = true;
                    }
                    else
                    {
                        g_gear_txt_digit = is_gear_inc ? (g_gear_txt_digit + STEPPER) : (g_gear_txt_digit - STEPPER);
                    }
                    return "gear_change";
                });
#endif
#endif


        }
        else  //this branch exec the high priority event
        {
            high_pri_q.push(needle_move);
        }
#endif

#endif
    });

#ifdef HAVE_LIBPLANES
    egt::PeriodicTimer scale_timer(std::chrono::milliseconds(16));
    scale_timer.on_timeout([&]()
    {
        //scale lambda function definition
        auto show_scale = [&]()
        {
            if (2.2 <= scale_factor)
            {
                is_scale_rev = true;
                is_scale_2_max = true;
#ifdef SCALE_TO_MAX_AND_STAY
                //gettimeofday(&time2, NULL);
                //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                //std::cout << "Scaling animation time: " << timediff << " us" << std::endl;
                is_scale_rev = false;
                is_scale_finish = true;
                scale_s.hide();
                scale_timer.stop();
                DeserialNeedles();
                timer.start();
                return;
#endif
            }
            if (0.001 >= scale_factor)
            {
                //gettimeofday(&time2, NULL);
                //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
                //std::cout << "Scaling animation time: " << timediff << " us" << std::endl;
                is_scale_rev = false;
                is_scale_finish = true;
                scale_s.hide();
                scale_timer.stop();
                DeserialNeedles();
                timer.start();
                return;
            }
            plane_set_pos(scale_ovl->s(), PLANE_WIDTH_HF - plane_width(scale_ovl->s())*scale_factor/2, PLANE_HEIGHT_HF - plane_height(scale_ovl->s())*scale_factor/2);
            plane_set_scale(scale_ovl->s(), scale_factor);
            plane_apply(scale_ovl->s());
            scale_factor = is_scale_rev ? scale_factor - 0.08 : scale_factor + 0.08;
        };

        //scale one step in every time out
        if (!is_scale_finish)
        {
            show_scale();
            return;
        }
    });
    scale_timer.start();
#else
    DeserialNeedles();
    timer.start();
#endif

#if 0  //use idle callback
    app->event().add_idle_callback([&]()
    {
        if (!low_pri_q.empty())
        {
            low_pri_q.front()();
            low_pri_q.pop();
        }
    });
#endif

//#else  //debug by touch screen
    auto handle_touch = [&](egt::Event & event)
    {
        switch (event.id())
        {
            case egt::EventId::pointer_click:
            case egt::EventId::keyboard_up:
            {
                //std::cout << "click..." << std::endl;
                if (motordash.get_gear_deserial_state())
                {
                    //std::cout << "push gear to low q" << std::endl;
                    low_pri_q.push([&]()
                    {
                        motordash.hide_gear_text(GearBase);
                        gear_index = (5 <= gear_index) ? 0 : gear_index;
                        //std::cout << "gear: " << gear_index << std::endl;
                        GearBase[gear_index++]->show();
                        return "click_gear";
                    });
                }
                break;
            }
            case egt::EventId::pointer_drag_start:
                //cout << "pointer_drag_start" << endl;

                break;
            case egt::EventId::pointer_drag_stop:

            break;
            case egt::EventId::pointer_drag:
            {
                //cout << "pointer_drag" << endl;
                break;
            }
            default:
                break;
        }
    };
    window.on_event(handle_touch);
#endif

    //gettimeofday(&time2, NULL);
    //timediff = (time1.tv_sec < time2.tv_sec) ? (time2.tv_usec + 1000000 - time1.tv_usec) : (time2.tv_usec - time1.tv_usec);
    //std::cout << "app start-up time: " << timediff << " us" << std::endl;
    //gettimeofday(&time1, NULL);
    
    signalI2C();
    return app.run();
}

// set the I2C signal
void signalI2C()
{
    int file;
    char buff[10];
    
    // open the i2c
    file = open("/dev/i2c-1", O_RDWR);
    if (file <0) {
        std::cout << "Failed to open /dev/i2c-1" << std::endl;
        return;
    }
    
    // specify the slave address
    int addr = 0x20;
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        close(file);
    	std::cout << "Failed to specify a valid I2C slave address" << std::endl;
        return;
    }    
    
    // force GP6 output high
    buff[0] = 0x09;
    buff[1] = 0x40;
    if (write(file, buff, 2) != 2) {
        // failed to write output high
        close(file);
        return;
    }
       
    // set the pins all as outputs
    buff[0] = 0x00; 	// select IODIR register 
    buff[1] = 0x00;	// set to all outputs
    if (write(file, buff, 2) != 2) {
        // failed to set IODIR register
        close(file);
        return;
    }
    
    // force GP6 output low
    buff[0] = 0x09;
    buff[1] = 0x00;
    if (write(file, buff, 2) != 2) {
        // failed to write output
        close(file);
        return;
    }
    
    close(file);
}






