/*
 * Copyright (C) 2018 Microchip Technology Inc.  All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <egt/ui>

int main(int argc, char** argv)
{
    egt::Application app(argc, argv);
#ifdef EXAMPLEDATA
    egt::add_search_path(EXAMPLEDATA);
#endif

    egt::TopWindow win;
    win.color(egt::Palette::ColorId::bg, egt::Palette::black);

    egt::VerticalBoxSizer sizer;
    win.add(expand(sizer));

    egt::ImageLabel logo(egt::Image("icon:egt_logo_white.png;128"));
    logo.margin(10);
    sizer.add(top(center(logo)));

    egt::StaticGrid grid(egt::StaticGrid::GridSize(3, 3));
    sizer.add(expand(grid));

    const std::pair<const char*, const char*> drums[] =
    {
        {"file:Closed-Hi-Hat-1.wav", "Hi-Hat"},
        {"file:Crash-Cymbal-1.wav", "Symbol"},
        {"file:Ensoniq-ESQ-1-Hi-Synth-Tom.wav", "Tom"},
        {"file:Alesis-Sanctuary-QCard-Loose-Bell-C5.wav", "Bell"},
        {"file:Ensoniq-ESQ-1-Snare.wav", "Snare"},
        {"file:Bass-Drum-1.wav", "Bass"},
    };

    for (auto& drum : drums)
    {
        auto button = std::make_shared<egt::Button>(drum.second);
        button->color(egt::Palette::ColorId::button_bg, egt::Palette::purple);
        button->color(egt::Palette::ColorId::button_bg, egt::Palette::purple, egt::Palette::GroupId::active);
        button->border(2);
        button->color(egt::Palette::ColorId::border, egt::Palette::black);
        button->color(egt::Palette::ColorId::border, egt::Palette::red, egt::Palette::GroupId::active);
        button->resize(egt::Size(100, 100));

        auto sound = std::make_shared<egt::experimental::Sound>(drum.first);
        button->on_event([sound](egt::Event&)
        {
            sound->play();
        }, {egt::EventId::raw_pointer_down});

        grid.add(egt::center(button));
    }

    win.show();

    return app.run();
}
