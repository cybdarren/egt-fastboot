/*
 * Copyright (C) 2018 Microchip Technology Inc.  All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "detail/dump.h"
#include "egt/detail/math.h"
#include "egt/input.h"
#include "egt/painter.h"
#include "egt/view.h"

namespace egt
{
inline namespace v1
{
ScrolledView::ScrolledView(Policy horizontal_policy,
                           Policy vertical_policy) noexcept
    : ScrolledView(Rect(), horizontal_policy, vertical_policy)
{
}

ScrolledView::ScrolledView(const Rect& rect,
                           Policy horizontal_policy,
                           Policy vertical_policy) noexcept
    : Frame(rect),
      m_hslider(100, 0, 0, Orientation::horizontal),
      m_vslider(100, 0, 0, Orientation::vertical),
      m_horizontal_policy(horizontal_policy),
      m_vertical_policy(vertical_policy)
{
    name("ScrolledView" + std::to_string(m_widgetid));

    m_hslider.slider_flags().set({Slider::SliderFlag::rectangle_handle,
                                  Slider::SliderFlag::consistent_line});

    m_vslider.slider_flags().set({Slider::SliderFlag::rectangle_handle,
                                  Slider::SliderFlag::consistent_line});

    resize_slider();
}

ScrolledView::ScrolledView(Frame& parent, const Rect& rect,
                           Policy horizontal_policy,
                           Policy vertical_policy) noexcept
    : ScrolledView(rect, horizontal_policy, vertical_policy)
{
    parent.add(*this);
}

ScrolledView::ScrolledView(Frame& parent,
                           Policy horizontal_policy,
                           Policy vertical_policy) noexcept
    : ScrolledView(horizontal_policy, vertical_policy)
{
    parent.add(*this);
}

ScrolledView::ScrolledView(Serializer::Properties& props) noexcept
    : Frame(props),
      m_hslider(100, 0, 0, Orientation::horizontal),
      m_vslider(100, 0, 0, Orientation::vertical)
{
    name("ScrolledView" + std::to_string(m_widgetid));

    m_hslider.slider_flags().set({Slider::SliderFlag::rectangle_handle,
                                  Slider::SliderFlag::consistent_line});

    m_vslider.slider_flags().set({Slider::SliderFlag::rectangle_handle,
                                  Slider::SliderFlag::consistent_line});

    resize_slider();
}

void ScrolledView::draw(Painter& painter, const Rect& rect)
{
    if (!m_canvas)
        return;

    //
    // All children are drawn to the internal m_canvas.  Then, the proper part
    // of the canvas is drawn based on of the m_offset.
    //
    Painter cpainter(m_canvas->context());

    Rect crect = to_child(super_rect());

    Palette::GroupId group = Palette::GroupId::normal;
    if (disabled())
        group = Palette::GroupId::disabled;
    else if (active())
        group = Palette::GroupId::active;

    /*
     * Children are drawn on a canvas which is a cairo surface. This
     * surface will be copied to the composition surface.
     * If the canvas has no background and a child as well, it can overlap the
     * previous drawing. For instance, the child is a Label with no background,
     * the new text will be drawn on the canvas then copied on the composition
     * buffer overlapping the previous text. If there is a background for the
     * canvas, it will 'erase' the previous text.
     * To prevent this situation, we force the background drawing of the
     * ScrolledView even if fill flags are set to none.
     */
    cpainter.set(color(Palette::ColorId::bg, group));
    theme().draw_box(cpainter,
                     Theme::FillFlag::solid,
                     crect /*m_canvas->size()*//*to_child(box())*/,
                     color(Palette::ColorId::border, group),
                     color(Palette::ColorId::bg, group),
                     border(),
                     margin());

    for (auto& child : m_children)
    {
        if (!child->visible())
            continue;

        // don't draw plane frame as child - this is
        // specifically handled by event loop
        if (child->plane_window())
            continue;

        if (child->box().intersect(crect))
        {
            // don't give a child a rectangle that is outside of its own box
            const auto r = Rect::intersection(crect, child->box());
            if (r.empty())
                continue;

            {
                // no matter what the child draws, clip the output to only the
                // rectangle we care about updating
                Painter::AutoSaveRestore sr2(cpainter);
                if (clip())
                {
                    cpainter.draw(r);
                    cpainter.clip();
                }

                detail::code_timer(false, child->name() + " draw: ", [&]()
                {
                    child->draw(cpainter, r);
                });
            }

            special_child_draw(cpainter, child.get());
        }
    }

    // change origin to paint canvas area and sliders

    Painter::AutoSaveRestore sr(painter);

    Point origin = point();
    if (origin.x() || origin.y())
    {
        //
        // Origin about to change
        //
        auto cr = painter.context();
        cairo_translate(cr.get(),
                        origin.x(),
                        origin.y());
    }

    // limit to content area
    const auto mrect = Rect::intersection(to_child(box()), to_child(content_area()));

    /*
     * We really want a basic copy here. We don't want to take into account the
     * previous drawing.
     */
    cairo_set_operator(painter.context().get(), CAIRO_OPERATOR_SOURCE);
    cairo_set_source_surface(painter.context().get(), m_canvas->surface().get(),
                             m_offset.x(), m_offset.y());
    cairo_rectangle(painter.context().get(),
                    mrect.point().x(), mrect.point().y(), mrect.width(), mrect.height());
    painter.fill();

    if (hscrollable())
        m_hslider.draw(painter, rect);
    if (vscrollable())
        m_vslider.draw(painter, rect);
}

void ScrolledView::resize(const Size& size)
{
    Frame::resize(size);
    resize_slider();
}

void ScrolledView::layout()
{
    Frame::layout();

    if (!visible())
        return;

    // we cannot layout with no space
    if (size().empty())
        return;

    if (m_in_layout)
        return;

    m_in_layout = true;
    auto reset = detail::on_scope_exit([this]() { m_in_layout = false; });

    bool hold = hscrollable();
    bool vold = vscrollable();

    update_scrollable();

    if (hold != hscrollable() || vold != vscrollable())
    {
        resize_slider();
        damage();
    }

    update_sliders();

    auto s = super_rect().size();

    if (!m_canvas || m_canvas->size() != s)
    {
        m_canvas = std::make_unique<Canvas>(s);
        damage();
    }
}

void ScrolledView::resize_slider()
{
    if (hscrollable())
    {
        auto b = box();
        b.y(b.y() + b.height() - m_slider_dim);
        b.height(m_slider_dim);

        if (vscrollable())
            b.width(b.width() - m_slider_dim);

        m_hslider.move(b.point() - point());
        m_hslider.resize(b.size());
    }

    if (vscrollable())
    {
        auto b = box();
        b.x(b.x() + b.width() - m_slider_dim);
        b.width(m_slider_dim);

        if (hscrollable())
            b.height(b.height() - m_slider_dim);

        m_vslider.move(b.point() - point());
        m_vslider.resize(b.size());
    }
}

Rect ScrolledView::super_rect() const
{
    auto result = box();
    for (const auto& child : m_children)
    {
        result = Rect::merge(result, child->to_parent(child->box()));
    }
    return result;
}

Point ScrolledView::offset_max() const
{
    const auto super = super_rect();
    return Point(super.width() - content_area().width(),
                 super.height() - content_area().height()) * -1;
}

void ScrolledView::offset(Point offset)
{
    if (hscrollable() || vscrollable())
    {
        auto offmax = offset_max();
        if (offset.x() > 0)
            offset.x(0);
        else if (offset.x() < offmax.x())
            offset.x(offmax.x());

        if (offset.y() > 0)
            offset.y(0);
        else if (offset.y() < offmax.y())
            offset.y(offmax.y());

        if (detail::change_if_diff<>(m_offset, offset))
        {
            update_sliders();
            damage();
        }
    }
}

void ScrolledView::update_sliders()
{
    const auto offmax = offset_max();

    if (offmax.x() < 0)
    {
        const auto hslider_value =
            egt::detail::normalize<float>(std::abs(m_offset.x()), 0, -offmax.x(), 0, 100);
        if (!detail::float_equal(m_hslider.value(hslider_value), hslider_value))
            damage();
    }

    if (offmax.y() < 0)
    {
        const auto vslider_value =
            egt::detail::normalize<float>(std::abs(m_offset.y()), 0, -offmax.y(), 0, 100);
        if (!detail::float_equal(m_vslider.value(vslider_value), vslider_value))
            damage();
    }
}

void ScrolledView::handle(Event& event)
{
    switch (event.id())
    {
    case EventId::pointer_drag_start:
        m_start_offset = m_offset;
        break;
    case EventId::pointer_drag:
    {
        auto diff = event.pointer().point -
                    event.pointer().drag_start;
        offset(m_start_offset + Point(diff.x(), diff.y()));
        break;
    }
    default:
        break;
    }

    switch (event.id())
    {
    case EventId::raw_pointer_down:
    case EventId::raw_pointer_up:
    case EventId::raw_pointer_move:
    case EventId::pointer_click:
    case EventId::pointer_dblclick:
    case EventId::pointer_hold:
    case EventId::pointer_drag_start:
    case EventId::pointer_drag:
    case EventId::pointer_drag_stop:
    {
        /*
         * Take into account the offset to get the real position of the pointer
         * in the ScrolledView.
         */
        Point pos = display_to_local(event.pointer().point) - m_offset;

        for (auto& child : detail::reverse_iterate(m_children))
        {
            if (child->readonly())
                continue;

            if (child->disabled())
                continue;

            if (!child->visible())
                continue;

            if (child->box().intersect(pos))
            {
                /*
                 * The pointer position must be updated before delegating the
                 * event handling to the child.
                 */
                event.pointer().point -= DisplayPoint(m_offset);
                child->handle(event);
                if (event.quit())
                    return;
            }
        }

        break;
    }
    default:
        break;
    }
}

}
}
