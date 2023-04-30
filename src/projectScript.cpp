//--------------------------------------------------
// Box Pushing
// projectScript.cpp
// Date: 2023-04-30
//--------------------------------------------------
#include "projectScript.h"
#include "imgui.h"
#include <atta/component/components/components.h>
#include <atta/component/interface.h>
#include <atta/graphics/interface.h>
#include <atta/resource/interface.h>
#include <atta/utils/namespaces.h>

cmp::Entity walls(1);

void ProjectScript::onLoad() {}

void ProjectScript::onUnload() { resetMap(); }

void ProjectScript::onStart() {}

void ProjectScript::onStop() {}

void ProjectScript::onAttaLoop() {}

void ProjectScript::onUIRender() {
    ImGui::SetNextWindowSize(ImVec2(310, 300), ImGuiCond_Once);
    ImGui::Begin("Project");
    {
        static std::vector<std::string> maps; // Available maps
        static int selected = 0;              // Selected map
        bool firstTime = false;

        // Populate maps
        if (maps.empty()) {
            firstTime = true;
            for (const auto& entry : std::filesystem::directory_iterator("resources/maps/"))
                if (entry.path().extension() == ".png")
                    maps.push_back(entry.path().stem());
            std::sort(maps.begin(), maps.end());
        }

        // Combo to select map
        if (ImGui::Combo(
                "Map", &selected,
                [](void* data, int index, const char** out_text) {
                    auto& maps = *reinterpret_cast<std::vector<std::string>*>(data);
                    if (index < 0 || index >= static_cast<int>(maps.size())) {
                        return false;
                    }
                    *out_text = maps[index].c_str();
                    return true;
                },
                reinterpret_cast<void*>(&maps), static_cast<int>(maps.size())) ||
            firstTime) {
            loadMap(maps[selected]);
        }

        // Preview
        atta::StringId sid = "maps/" + maps[selected] + ".png";
        ImGui::Image(gfx::getImGuiImage(sid), ImVec2(50, 50));
    }
    ImGui::End();
}

struct Box {
    atta::vec2i pos;
    atta::vec2i size;
};

std::ostream& operator<<(std::ostream& os, const Box& box) {
    os << "{" << box.pos.x << "x" << box.pos.y << " " << box.size.x << "x" << box.size.y << "}";
    return os;
}

std::vector<Box> parseBoxes(std::vector<bool> bin, uint32_t w, uint32_t h) {
    std::vector<Box> boxes;

    // Start creating one box for each pixel
    for (uint32_t y = 0; y < h; y++)
        for (uint32_t x = 0; x < w; x++)
            if (bin[y * w + x])
                boxes.push_back({atta::vec2i(x, y), atta::vec2i(1, 1)});

    // Merge boxes when possible
    bool boxesMerged = true;
    while (true) {
        boxesMerged = true;
        for (int i = 0; i < boxes.size(); i++) {
            for (int j = 0; j < boxes.size(); j++) {
                if (i == j)
                    continue;
                // Merge vertical
                if (boxes[i].size.x == boxes[j].size.x && boxes[i].pos.x == boxes[j].pos.x && boxes[i].pos.y + boxes[i].size.y == boxes[j].pos.y) {
                    boxes[i].size.y += boxes[j].size.y;
                    boxes.erase(boxes.begin() + j);
                    boxesMerged = false;
                    goto nextCheck;
                }

                // Merge horizontal
                if (boxes[i].size.y == boxes[j].size.y && boxes[i].pos.y == boxes[j].pos.y && boxes[i].pos.x + boxes[i].size.x == boxes[j].pos.x) {
                    boxes[i].size.x += boxes[j].size.x;
                    boxes.erase(boxes.begin() + j);
                    boxesMerged = false;
                    goto nextCheck;
                }
            }
        }
    nextCheck:
        if (boxesMerged)
            break;
    }

    return boxes;
}

void ProjectScript::loadMap(std::string name) {
    resetMap();

    // Get image info
    res::Image* mapImg = res::get<res::Image>("maps/" + name + ".png");
    uint32_t w = mapImg->getWidth();
    uint32_t h = mapImg->getHeight();
    uint32_t c = mapImg->getChannels();
    uint8_t* data = mapImg->getData();

    // Parse image to create walls
    std::vector<bool> bin(w * h);
    for (int i = 0, j = 0; i < w * h * c; i += c, j++)
        bin[j] = data[i] > 127 ? false : true;
    std::vector<Box> boxes = parseBoxes(bin, w, h);

    // Create parsed boxes
    for (const Box& box : boxes) {
        cmp::Entity wall = cmp::createEntity();

        atta::vec2 pos = atta::vec2(box.pos.x, box.pos.y) / 10.0f;
        atta::vec2 size = atta::vec2(box.size.x, box.size.y) / 10.0f;
        pos += size / 2;

        cmp::Transform* t = wall.add<cmp::Transform>();
        t->position = atta::vec3(pos, 0.25f);
        t->scale = atta::vec3(size, 0.5f);
        t->orientation.set2DAngle(0.0f);

        wall.add<cmp::Mesh>()->set("meshes/cube.obj");

        wall.add<cmp::Material>()->set("wall");

        wall.add<cmp::Name>()->set("Wall");

        auto rb = wall.add<cmp::RigidBody2D>();
        rb->type = cmp::RigidBody2D::Type::STATIC;
        rb->friction = 0.0f;
        wall.add<cmp::BoxCollider2D>();

        walls.get<cmp::Relationship>()->addChild(walls, wall);
    }

    LOG_SUCCESS("ProjectScript", "Map [w]$0[] loaded successfully", name);
}

void ProjectScript::resetMap() {
    for (cmp::Entity child : walls.getChildren())
        cmp::deleteEntity(child);
}
