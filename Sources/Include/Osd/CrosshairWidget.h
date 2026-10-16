#pragma once

#include "OsdWidget.h"

struct CrosshairWidgetData {};

class CrosshairWidget : public OsdWidget<CrosshairWidget, CrosshairWidgetData, 1, 1>
{
    using Base = OsdWidget<CrosshairWidget, CrosshairWidgetData, 1, 1>;

public:
    using Content = typename Base::Content;

    CrosshairWidget()
        : Base({})
    {
    }

    Content RenderContent(const CrosshairWidgetData&) const
    {
        return {{{OsdChar::AhCenter}}};
    }
};
