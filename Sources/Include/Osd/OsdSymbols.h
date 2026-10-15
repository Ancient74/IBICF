#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

enum class OsdChar : uint8_t {
    // ---- Row 0x0_ -------------------------------------------------------
    Null            = 0x00,
    Rssi            = 0x01,
    AhBracketLeft   = 0x02,
    AhBracketRight  = 0x03,
    Throttle        = 0x04,
    ThrottleArrow   = 0x05,
    Volt            = 0x06,
    MilliampHour    = 0x07,
    MarkerTop       = 0x08,
    MarkerMiddle    = 0x09,
    MarkerBottpm    = 0x0A,
    Crosshair       = 0x0B,
    Meter           = 0x0C,
    DegreesF        = 0x0D,
    DegreesC        = 0x0E,
    Feet            = 0x0F,

    // ---- Row 0x1_ -------------------------------------------------------
    Blackbox        = 0x10,
    Home            = 0x11,
    Rpm             = 0x12,
    AhDecoration    = 0x13,
    Roll            = 0x14,
    Pitch           = 0x15,
    LineVertical    = 0x16,
    LineHorizontal  = 0x17,
    HeadingN        = 0x18,
    HeadingS        = 0x19,
    HeadingE        = 0x1A,
    HeadingW        = 0x1B,
    HeadingLineDivided = 0x1C,
    HeadingLine     = 0x1D,
    SatelliteLeft   = 0x1E,
    SatelliteRight  = 0x1F,

    // ---- Row 0x2_ -------------------------------------------------------
    Space           = 0x20,
    Exclamation     = 0x21,
    Quote           = 0x22,
    Hash            = 0x23,
    Max             = 0x24,  // takes the place of '$'
    Percent         = 0x25,
    AhCenterLineLeft  = 0x26,
    AhCenterLineRight = 0x27,
    LeftParen       = 0x28,
    RightParen      = 0x29,
    Asterisk        = 0x2A,
    Plus            = 0x2B,
    Comma           = 0x2C,
    Minus           = 0x2D,
    Period          = 0x2E,
    Slash           = 0x2F,

    // ---- Row 0x3_ -------------------------------------------------------
    Digit0 = 0x30, Digit1 = 0x31, Digit2 = 0x32, Digit3 = 0x33, Digit4 = 0x34,
    Digit5 = 0x35, Digit6 = 0x36, Digit7 = 0x37, Digit8 = 0x38, Digit9 = 0x39,
    Colon           = 0x3A,
    Semicolon       = 0x3B,
    Less            = 0x3C,
    Equals          = 0x3D,
    Greater         = 0x3E,
    Question        = 0x3F,

    // ---- Row 0x4_ / 0x5_ ------------------------------------------------
    At              = 0x40,
    A = 0x41, B = 0x42, C = 0x43, D = 0x44, E = 0x45, F = 0x46, G = 0x47,
    H = 0x48, I = 0x49, J = 0x4A, K = 0x4B, L = 0x4C, M = 0x4D, N = 0x4E, O = 0x4F,
    P = 0x50, Q = 0x51, R = 0x52, S = 0x53, T = 0x54, U = 0x55, V = 0x56,
    W = 0x57, X = 0x58, Y = 0x59, Z = 0x5A,
    LeftBracket     = 0x5B,
    Backslash       = 0x5C,
    RightBracket    = 0x5D,
    Caret           = 0x5E,
    Underscore      = 0x5F,

    // ---- Row 0x6_ : 16-direction arrows (screen directions) -------------
    ArrowS   = 0x60, ArrowSSE = 0x61, ArrowSE  = 0x62, ArrowESE = 0x63,
    ArrowE   = 0x64, ArrowENE = 0x65, ArrowNE  = 0x66, ArrowNNE = 0x67,
    ArrowN   = 0x68, ArrowNNW = 0x69, ArrowNW  = 0x6A, ArrowWNW = 0x6B,
    ArrowW   = 0x6C, ArrowWSW = 0x6D, ArrowSW  = 0x6E, ArrowSSW = 0x6F,

    // ---- Row 0x7_ -------------------------------------------------------
    OnHours         = 0x70,
    FlyHours        = 0x71,
    CornerBottomRight = 0x72,
    CornerTopRight  = 0x73,
    CornerBottomLeft = 0x74,
    ArrowSmallUp    = 0x75,
    ArrowSmallDown  = 0x76,
    ArrowSmallRight = 0x77,
    ArrowSmallLeft  = 0x78,
    CornerTopLeft   = 0x79,
    Thermometer     = 0x7A,
    VerticalBar     = 0x7C,
    AhCenter        = 0x7E,
    Altitude        = 0x7F,

    // ---- Row 0x8_ : artificial horizon bar + progress bar ---------------
    AhBar0 = 0x80, AhBar1 = 0x81, AhBar2 = 0x82, AhBar3 = 0x83, AhBar4 = 0x84,
    AhBar5 = 0x85, AhBar6 = 0x86, AhBar7 = 0x87, AhBar8 = 0x88,
    Latitude        = 0x89,
    ProgressStart   = 0x8A,
    ProgressFull    = 0x8B,
    ProgressHalf    = 0x8C,
    ProgressEmpty   = 0x8D,
    ProgressEnd     = 0x8E,
    ProgressClose   = 0x8F,

    // ---- Row 0x9_ : battery, units --------------------------------------
    BatteryFull     = 0x90,
    Battery5        = 0x91,
    Battery4        = 0x92,
    Battery3        = 0x93,
    Battery2        = 0x94,
    Battery1        = 0x95,
    BatteryEmpty    = 0x96,
    MainBattery     = 0x97,
    Longitude       = 0x98,
    FeetPerSecond   = 0x99,
    Amp             = 0x9A,
    OnMinutes       = 0x9B,
    FlyMinutes      = 0x9C,
    Mph             = 0x9D,
    Kph             = 0x9E,
    MetersPerSecond = 0x9F,
};

// Converts one ASCII character to its OSD font code.
//  - digits, letters and the common punctuation of the map are supported
//  - lowercase letters map to uppercase glyphs (the font has no lowercase)
//  - anything else (including '$', '&', '\'') becomes Space
constexpr OsdChar OsdCharFromChar(char c) noexcept {
    if (c >= '0' && c <= '9') return static_cast<OsdChar>(static_cast<uint8_t>(OsdChar::Digit0) + (c - '0'));
    if (c >= 'A' && c <= 'Z') return static_cast<OsdChar>(static_cast<uint8_t>(OsdChar::A) + (c - 'A'));
    if (c >= 'a' && c <= 'z') return static_cast<OsdChar>(static_cast<uint8_t>(OsdChar::A) + (c - 'a'));

    switch (c) {
        case ' ':  return OsdChar::Space;
        case '!':  return OsdChar::Exclamation;
        case '"':  return OsdChar::Quote;
        case '#':  return OsdChar::Hash;
        case '%':  return OsdChar::Percent;
        case '(':  return OsdChar::LeftParen;
        case ')':  return OsdChar::RightParen;
        case '*':  return OsdChar::Asterisk;
        case '+':  return OsdChar::Plus;
        case ',':  return OsdChar::Comma;
        case '-':  return OsdChar::Minus;
        case '.':  return OsdChar::Period;
        case '/':  return OsdChar::Slash;
        case ':':  return OsdChar::Colon;
        case ';':  return OsdChar::Semicolon;
        case '<':  return OsdChar::Less;
        case '=':  return OsdChar::Equals;
        case '>':  return OsdChar::Greater;
        case '?':  return OsdChar::Question;
        case '@':  return OsdChar::At;
        case '[':  return OsdChar::LeftBracket;
        case '\\': return OsdChar::Backslash;
        case ']':  return OsdChar::RightBracket;
        case '^':  return OsdChar::Caret;
        case '_':  return OsdChar::Underscore;
        default:   return OsdChar::Space;
    }
}

template <std::size_t N>
constexpr std::array<OsdChar, N - 1> EncodeToOsdChar(const char (&str)[N]) noexcept
{
    std::array<OsdChar, N - 1> out{};
    for (std::size_t i = 0; i < N - 1; ++i)
        out[i] = FromChar(str[i]);

    return out;
}
