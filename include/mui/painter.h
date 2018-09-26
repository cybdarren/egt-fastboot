/*
 * Copyright (C) 2018 Microchip Technology Inc.  All rights reserved.
 * Joshua Henderson <joshua.henderson@microchip.com>
 */
#ifndef MUI_PAINTER_H
#define MUI_PAINTER_H

#include <mui/font.h>
#include <mui/palette.h>
#include <mui/geometry.h>
#include <cairo.h>
#include <memory>

namespace mui
{

    using shared_cairo_surface_t =
        std::shared_ptr<cairo_surface_t>;

    using shared_cairo_t =
        std::shared_ptr<cairo_t>;

    /**
     * Drawing interface for graphics.
     *
     * <https://www.cairographics.org/manual/index.html>
     */
    class Painter
    {
    public:

        Painter();

        Painter(shared_cairo_t cr);

        virtual ~Painter();

        void begin();

        void end();

        Painter& set_color(const Color& color);

        Painter& draw_rectangle(const Rect& rect);

        Painter& draw_fillrectangle(const Rect& rect);

        Painter& set_line_width(float width);

        Painter& draw_line(const Point& start, const Point& end);

        Painter& draw_line(const Point& start, const Point& end, float width);

        Painter& draw_image(const Point& point, shared_cairo_surface_t surface, bool bw = false);

        Painter& draw_image(const Rect& rect, const Point& point, shared_cairo_surface_t surface);

        Painter& draw_arc(const Point& point, float radius, float angle1, float angle2);

        /*
          void stroke()
          {
          cairo_stroke(m_cr.get());
          }

          void paint()
          {
          cairo_paint(m_cr.get());
          }

          void fill()
          {
          cairo_fill(m_cr.get());
          }
        */

        /**
         * Set the active font.
         */
        Painter& set_font(const Font& font);

        /**
         * Draw text at the specified point.
         */
        Painter& draw_text(const Point& point, const std::string& str);

        /**
         * Draw text aligned inside the specified rectangle.
         */
        Painter& draw_text(const Rect& rect, const std::string& str, int align, int standoff = 5);

        shared_cairo_t context() const
        {
            return m_cr;
        }

        void paint_surface_with_drop_shadow(cairo_surface_t* source_surface,
                                            int shadow_offset,
                                            double shadow_alpha,
                                            double tint_alpha,
                                            int srx,
                                            int srcy,
                                            int width,
                                            int height,
                                            int dstx,
                                            int dsty);


    protected:

        shared_cairo_t m_cr;
    };

}

#endif
