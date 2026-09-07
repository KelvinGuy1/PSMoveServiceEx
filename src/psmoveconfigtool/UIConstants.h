#ifndef UI_CONSTANTS_H
#define UI_CONSTANTS_H

static const float k_background_alpha = 0.65f;

#define UI_ARRAYSIZE(_ARR)      ((int)(sizeof(_ARR)/sizeof(*_ARR)))

#include <imgui.h>

namespace ImGui
{
    inline bool ButtonChecked(const char* label, bool checked, const ImVec2 &size = ImVec2(0, 0))
    {
        if (checked)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_ButtonActive]);
        bool pressed = ImGui::Button(label, size);
        if (checked)
            ImGui::PopStyleColor();
        return pressed;
    }
}

#endif // UI_CONSTANTS_H
