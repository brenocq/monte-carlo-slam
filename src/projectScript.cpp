//--------------------------------------------------
// Box Pushing
// projectScript.cpp
// Date: 2023-04-30
//--------------------------------------------------
#include "projectScript.h"
#include "imgui.h"
#include <atta/utils/namespaces.h>

void ProjectScript::onLoad() { LOG_DEBUG("ProjectScript", "onLoad"); }

void ProjectScript::onUnload() { LOG_DEBUG("ProjectScript", "onUnload"); }

void ProjectScript::onStart() { LOG_DEBUG("ProjectScript", "onStart"); }

void ProjectScript::onStop() { LOG_DEBUG("ProjectScript", "onStop"); }

void ProjectScript::onAttaLoop() { LOG_DEBUG("ProjectScript", "onAttaLoop"); }

void ProjectScript::onUIRender() {
    LOG_DEBUG("ProjectScript", "onUIRender");
    ImGui::SetNextWindowSize(ImVec2(310, 300), ImGuiCond_Once);
    ImGui::Begin("Project");
    { 
        ImGui::Text("Hi"); 
        ImGui::Separator();
    }
    ImGui::End();
}
