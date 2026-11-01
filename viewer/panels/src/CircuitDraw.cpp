// Symbols of a circuit; see CircuitDraw.hpp. Each bus is a rail with its branches hanging down to
// ground, batteries first, as a schematic is read: current leaves them along the rail.
#include "CircuitDraw.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

const ImVec4 kWire = theme::kDomainColor[static_cast<int>(theme::Domain::Electrical)];
const ImVec4 kRegen = c::info;

constexpr float kColumn = 168.0F;  // between branches
constexpr float kBranch = 200.0F;  // rail to ground
constexpr float kSymbol = 44.0F;   // height of one symbol
constexpr float kRailTop = 44.0F;  // room for the bus's name above its rail
constexpr float kBelow = 56.0F;    // under the grounds, before the next bus
constexpr float kPick = 150.0F;    // width of a branch's clickable area

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

struct Pen {
    ImDrawList* draw;
    ImU32 color;
    float width;
};

void resistor(const Pen& pen, float x, float top, float bottom) {
    // The zigzag fills the middle of the span; straight leads reach its ends.
    constexpr int kPeaks = 6;
    const float lead = (bottom - top) * 0.15F;
    const float amplitude = px(6.0F);
    const float step = (bottom - top - 2.0F * lead) / static_cast<float>(kPeaks);
    ImVec2 last(x, top);
    ImVec2 next(x, top + lead);
    pen.draw->AddLine(last, next, pen.color, pen.width);
    last = next;
    for (int i = 0; i < kPeaks; ++i) {
        const float side = i % 2 == 0 ? amplitude : -amplitude;
        next = ImVec2(x + side, top + lead + step * (static_cast<float>(i) + 0.5F));
        pen.draw->AddLine(last, next, pen.color, pen.width);
        last = next;
    }
    next = ImVec2(x, bottom - lead);
    pen.draw->AddLine(last, next, pen.color, pen.width);
    pen.draw->AddLine(next, ImVec2(x, bottom), pen.color, pen.width);
}

void capacitor(const Pen& pen, float x, float top, float bottom) {
    const float middle = (top + bottom) * 0.5F;
    const float gap = px(3.0F);
    const float half = px(9.0F);
    pen.draw->AddLine(ImVec2(x, top), ImVec2(x, middle - gap), pen.color, pen.width);
    pen.draw->AddLine(ImVec2(x - half, middle - gap), ImVec2(x + half, middle - gap), pen.color,
                      pen.width);
    pen.draw->AddLine(ImVec2(x - half, middle + gap), ImVec2(x + half, middle + gap), pen.color,
                      pen.width);
    pen.draw->AddLine(ImVec2(x, middle + gap), ImVec2(x, bottom), pen.color, pen.width);
}

/// The long plate is the positive side, on top.
void battery(const Pen& pen, float x, float top, float bottom) {
    const float middle = (top + bottom) * 0.5F;
    const float gap = px(3.0F);
    pen.draw->AddLine(ImVec2(x, top), ImVec2(x, middle - gap), pen.color, pen.width);
    pen.draw->AddLine(ImVec2(x - px(11.0F), middle - gap), ImVec2(x + px(11.0F), middle - gap),
                      pen.color, pen.width);
    pen.draw->AddLine(ImVec2(x - px(5.0F), middle + gap), ImVec2(x + px(5.0F), middle + gap),
                      pen.color, pen.width * 2.0F);
    pen.draw->AddLine(ImVec2(x, middle + gap), ImVec2(x, bottom), pen.color, pen.width);
}

/// A circle on the wire with a letter in it: a motor, or a load of constant power.
void circled(const Pen& pen, float x, float top, float bottom, const char* letter) {
    const float middle = (top + bottom) * 0.5F;
    const float radius = px(13.0F);
    pen.draw->AddLine(ImVec2(x, top), ImVec2(x, middle - radius), pen.color, pen.width);
    pen.draw->AddCircle(ImVec2(x, middle), radius, pen.color, 24, pen.width);
    pen.draw->AddLine(ImVec2(x, middle + radius), ImVec2(x, bottom), pen.color, pen.width);
    ImFont* font = theme::fonts().sansSemiBold;
    const float size = px(theme::kLabelSize);
    const ImVec2 extent = font->CalcTextSizeA(size, FLT_MAX, 0.0F, letter);
    pen.draw->AddText(font, size, ImVec2(x - extent.x * 0.5F, middle - extent.y * 0.5F), pen.color,
                      letter);
}

void ground(const Pen& pen, float x, float y) {
    const float step = px(4.0F);
    for (int i = 0; i < 3; ++i) {
        const float half = px(10.0F) - px(3.5F) * static_cast<float>(i);
        const float at = y + step * static_cast<float>(i);
        pen.draw->AddLine(ImVec2(x - half, at), ImVec2(x + half, at), pen.color, 1.0F);
    }
}

/// A resistor and a capacitor side by side on one span of the wire.
void pair(const Pen& pen, float x, float top, float bottom) {
    const float side = px(14.0F);
    pen.draw->AddLine(ImVec2(x - side, top), ImVec2(x + side, top), pen.color, pen.width);
    pen.draw->AddLine(ImVec2(x - side, bottom), ImVec2(x + side, bottom), pen.color, pen.width);
    resistor(pen, x - side, top, bottom);
    capacitor(pen, x + side, top, bottom);
}

void text(ImDrawList* draw, ImFont* font, float size, float x, float y, ImU32 color,
          const std::string& value) {
    draw->AddText(font, size, ImVec2(x, y), color, value.c_str());
}

/// A branch from the rail to ground, with its symbols in series and its name and values beside.
void branch(ImDrawList* draw, const CircuitPart& part, float x, float rail, double amperes,
            bool selected) {
    const bool known = !std::isnan(amperes);
    const bool regen = known && amperes < 0.0 && part.kind != CircuitPart::Kind::Battery;
    const bool charging = known && amperes < 0.0 && part.kind == CircuitPart::Kind::Battery;
    const ImVec4& wireColor = regen || charging ? kRegen : kWire;
    const Pen wirePen{draw, u32(wireColor), px(1.5F)};
    const Pen symbolPen{draw, u32(selected ? c::accent : c::t1), px(1.5F)};
    const float end = rail + px(kBranch);

    // The symbols of the branch, top to bottom.
    enum class Symbol : std::uint8_t { Resistor, Pair, Battery, Motor, Power, Converter };
    std::vector<Symbol> symbols;
    switch (part.kind) {
        case CircuitPart::Kind::Battery:
            symbols.push_back(Symbol::Resistor);
            symbols.insert(symbols.end(), part.pairs.size(), Symbol::Pair);
            symbols.push_back(Symbol::Battery);
            break;
        case CircuitPart::Kind::Motor:
            symbols.push_back(Symbol::Motor);
            break;
        case CircuitPart::Kind::PowerLoad:
            symbols.push_back(Symbol::Power);
            break;
        case CircuitPart::Kind::Converter:
            symbols.push_back(Symbol::Converter);
            break;
        default:
            symbols.push_back(Symbol::Resistor);
            break;
    }
    const float spare = px(kBranch) - px(kSymbol) * static_cast<float>(symbols.size());
    const float lead = std::max(spare * 0.5F, px(8.0F));
    float y = rail + lead;
    draw->AddLine(ImVec2(x, rail), ImVec2(x, y), wirePen.color, wirePen.width);
    for (const Symbol symbol : symbols) {
        const float next = y + px(kSymbol);
        switch (symbol) {
            case Symbol::Resistor:
                resistor(symbolPen, x, y, next);
                break;
            case Symbol::Pair:
                pair(symbolPen, x, y, next);
                break;
            case Symbol::Battery:
                battery(symbolPen, x, y, next);
                break;
            case Symbol::Motor:
                circled(symbolPen, x, y, next, "M");
                break;
            case Symbol::Converter:
                circled(symbolPen, x, y, next, "DC");
                break;
            case Symbol::Power:
                circled(symbolPen, x, y, next, "P");
                break;
        }
        y = next;
    }
    draw->AddLine(ImVec2(x, y), ImVec2(x, end), wirePen.color, wirePen.width);
    ground(Pen{draw, u32(c::t2), 1.0F}, x, end);

    const theme::Fonts& fonts = theme::fonts();
    const float label = px(theme::kLabelSize);
    const float small = px(theme::kCaptionSize);
    const float textX = x + px(24.0F);
    draw->PushClipRect(ImVec2(x - px(30.0F), rail), ImVec2(x + px(kColumn) - px(30.0F), end), true);
    text(draw, fonts.sansSemiBold, label, textX, rail + px(10.0F),
         u32(selected ? c::accent : c::t1), part.name);
    if (known) {
        char amps[32];
        std::snprintf(amps, sizeof(amps), "%.2f A", std::abs(amperes));
        text(draw, fonts.mono, small, textX, rail + px(28.0F), u32(wireColor), amps);
    }
    // One value per line, lowest on the branch last: they do not fit across a column.
    float row = rail + px(kBranch) * 0.5F;
    std::string values = partValues(part);
    if (part.kind == CircuitPart::Kind::Motor && !part.joint.empty()) {
        values += " · " + part.joint;
    }
    std::size_t from = 0;
    static constexpr std::string_view kBreak = " · ";
    while (from <= values.size() && !values.empty()) {
        const std::size_t to = values.find(kBreak, from);
        text(draw, fonts.mono, small, textX, row, u32(c::t2),
             values.substr(from, to == std::string::npos ? to : to - from));
        row += px(15.0F);
        if (to == std::string::npos) {
            break;
        }
        from = to + kBreak.size();
    }
    draw->PopClipRect();
}

/// A battery, or a converter on the bus it regulates.
bool feeds(const CircuitPart& part) {
    return part.kind == CircuitPart::Kind::Battery ||
           (part.kind == CircuitPart::Kind::Converter && !part.inputBus.empty());
}

/// The bus's capacitor as a branch of its own.
void capacitorBranch(ImDrawList* draw, const CircuitBus& bus, float x, float rail, bool selected) {
    const Pen wirePen{draw, u32(kWire), px(1.5F)};
    const Pen symbolPen{draw, u32(selected ? c::accent : c::t1), px(1.5F)};
    const float end = rail + px(kBranch);
    const float top = rail + (px(kBranch) - px(kSymbol)) * 0.5F;
    draw->AddLine(ImVec2(x, rail), ImVec2(x, top), wirePen.color, wirePen.width);
    capacitor(symbolPen, x, top, top + px(kSymbol));
    draw->AddLine(ImVec2(x, top + px(kSymbol)), ImVec2(x, end), wirePen.color, wirePen.width);
    ground(Pen{draw, u32(c::t2), 1.0F}, x, end);
    const theme::Fonts& fonts = theme::fonts();
    text(draw, fonts.mono, px(theme::kCaptionSize), x + px(24.0F), rail + px(kBranch) * 0.5F,
         u32(c::t2), engineering(bus.capacitance, "F"));
}

}  // namespace

std::optional<CircuitPick> drawCircuit(const Circuit& circuit, const PartCurrent& current,
                                       const BusVoltage& voltage,
                                       const std::optional<CircuitPick>& selected) {
    std::optional<CircuitPick> picked;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const theme::Fonts& fonts = theme::fonts();
    const float mono = px(theme::kMonoSize);
    float top = origin.y;
    float widest = 0.0F;
    for (std::size_t b = 0; b < circuit.buses.size(); ++b) {
        const CircuitBus& bus = circuit.buses[b];
        // What feeds it first, then the bus's capacitor, then what it feeds.
        std::vector<std::size_t> order;
        for (std::size_t p = 0; p < bus.parts.size(); ++p) {
            if (feeds(bus.parts[p])) {
                order.push_back(p);
            }
        }
        const std::size_t batteries = order.size();
        for (std::size_t p = 0; p < bus.parts.size(); ++p) {
            if (!feeds(bus.parts[p]) && bus.parts[p].kind != CircuitPart::Kind::Other) {
                order.push_back(p);
            }
        }
        const std::size_t columns = order.size() + (bus.capacitance > 0.0 ? 1 : 0);
        const float rail = top + px(kRailTop);
        const float left = origin.x + px(40.0F);
        const float right =
            left + px(kColumn) * static_cast<float>(std::max<std::size_t>(columns, 1) - 1);
        const bool busSelected = selected && selected->bus == b && !selected->part;
        draw->AddRectFilled(ImVec2(left - px(8.0F), rail - px(2.0F)),
                            ImVec2(right + px(8.0F), rail + px(2.0F)), u32(kWire), px(2.0F));
        // Above the rail: its name, voltage and limits.
        std::string title = bus.name;
        if (const double volts = voltage(b); !std::isnan(volts)) {
            char text2[32];
            std::snprintf(text2, sizeof(text2), "  %.2f V", volts);
            title += text2;
        }
        draw->AddText(fonts.mono, mono, ImVec2(left - px(8.0F), rail - px(24.0F)),
                      u32(busSelected ? c::accent : kWire), title.c_str());
        const std::string limits = busValues(bus);
        const float titleWidth = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, title.c_str()).x;
        draw->AddText(fonts.mono, px(theme::kCaptionSize),
                      ImVec2(left + titleWidth + px(14.0F), rail - px(23.0F)), u32(c::t3),
                      limits.c_str());
        ImGui::SetCursorScreenPos(ImVec2(left - px(8.0F), rail - px(26.0F)));
        ImGui::PushID(static_cast<int>(b));
        if (ImGui::InvisibleButton("##bus", ImVec2(std::max(titleWidth, px(60.0F)), px(26.0F)))) {
            picked = CircuitPick{b, std::nullopt};
        }
        float x = left;
        const auto column = [&](std::size_t index) {
            const std::size_t p = order[index];
            const bool isSelected = selected && selected->bus == b && selected->part == p;
            branch(draw, bus.parts[p], x, rail, current(b, p), isSelected);
            ImGui::SetCursorScreenPos(ImVec2(x - px(20.0F), rail + px(4.0F)));
            ImGui::PushID(static_cast<int>(p));
            if (ImGui::InvisibleButton("##part", ImVec2(px(kPick), px(kBranch)))) {
                picked = CircuitPick{b, p};
            }
            ImGui::PopID();
            x += px(kColumn);
        };
        for (std::size_t i = 0; i < batteries; ++i) {
            column(i);
        }
        if (bus.capacitance > 0.0) {
            capacitorBranch(draw, bus, x, rail, busSelected);
            x += px(kColumn);
        }
        for (std::size_t i = batteries; i < order.size(); ++i) {
            column(i);
        }
        ImGui::PopID();
        widest = std::max(widest, right + px(kColumn) - origin.x);
        top = rail + px(kBranch) + px(kBelow);
    }
    ImGui::SetCursorScreenPos(origin);
    ImGui::Dummy(ImVec2(widest, top - origin.y));
    return picked;
}

}  // namespace lagrange::viewer
