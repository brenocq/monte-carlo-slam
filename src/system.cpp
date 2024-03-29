//--------------------------------------------------
// Monte Carlo SLAM
// system.cpp
// Date: 2023-04-30
//--------------------------------------------------
#include "system.h"
#include "imgui.h"
#include "implot.h"
#include "mapComponent.h"
#include "robotComponent.h"
#include <atta/component/components/components.h>
#include <atta/component/interface.h>
#include <atta/graphics/drawer.h>
#include <atta/graphics/interface.h>
#include <atta/resource/interface.h>
#include <atta/utils/namespaces.h>

const cmp::Entity walls(1);
const cmp::Entity robot(7);
const cmp::Entity map(17);

void System::onLoad() { robot.get<RobotComponent>()->path = {}; }

void System::onUnload() { resetMap(); }

void System::onAttaLoop() {}

void plotState(std::string name, atta::vec2 pos, float angle) {
    atta::vec2 front = atta::vec2(cos(angle), sin(angle)) * 0.025;
    atta::vec2 p0 = pos - front;
    atta::vec2 p1 = pos + front;
    std::vector<float> x = {p0.x, p1.x};
    std::vector<float> y = {p0.y, p1.y};
    ImPlot::PlotLine(name.c_str(), x.data(), y.data(), x.size());
}

void System::onUIRender() {
    ImGui::SetNextWindowSize(ImVec2(310, 300), ImGuiCond_Once);
    ImGui::Begin("System");
    {
        static std::vector<std::string> maps; // Available maps
        static int selected = 1;              // Selected map
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

        // Robot grid preview
        int w = RobotComponent::width;
        int h = RobotComponent::height;
        float s = RobotComponent::cellSize;
        std::vector<float> grid(w * h);
        RobotComponent* r = robot.get<RobotComponent>();
        RobotComponent::Grid rGrid = r->grid;
        for (int i = 0; i < w * h; i++) {
            if (rGrid[i] == RobotComponent::FREE)
                grid[i] = 0.0f;
            else if (rGrid[i] == RobotComponent::UNKNOWN)
                grid[i] = 0.5f;
            else if (rGrid[i] == RobotComponent::COLLISION)
                grid[i] = 0.75f;
            else if (rGrid[i] == RobotComponent::WALL)
                grid[i] = 1.0f;
        }

        ImGui::SliderFloat("X", &r->goalPos.x, 0.0f, w * s);
        ImGui::SliderFloat("Y", &r->goalPos.y, 0.0f, h * s);

        if (ImPlot::BeginPlot("Robot grid view", nullptr, nullptr, ImVec2(600, 600))) {
            // Plot grid
            ImPlot::PushColormap(ImPlotColormap_Greys);
            ImPlot::PlotHeatmap("Occupancy", grid.data(), h, w, 0, 0, nullptr, ImPlotPoint(0, h * s), ImPlotPoint(w * s, 0));
            ImPlot::PopColormap();

            // Plot robot
            plotState("Robot", r->pos, r->ori);

            // Plot particles
            // for (RobotComponent::Particle particle : r->particles)
            //    plotState("Particles", particle.pos, particle.ori);

            // Plot path
            std::vector<float> x;
            std::vector<float> y;
            if (!r->path.empty()) {
                std::queue<atta::vec2> path = r->path;
                while (!path.empty()) {
                    atta::vec2 p = path.front();
                    path.pop();
                    x.push_back(p.x + s * 0.5f);
                    y.push_back(p.y + s * 0.5f);
                }
                ImPlot::PlotLine("Path", x.data(), y.data(), x.size());
            }

            // Goal
            ImVec2 center = ImPlot::PlotToPixels(ImPlotPoint(r->goalPos.x, r->goalPos.y));
            ImPlot::GetPlotDrawList()->AddCircleFilled(center, 5, IM_COL32(0, 255, 0, 255));

            ImPlot::EndPlot();
        }
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

std::vector<Box> parseBoxes(MapComponent::Grid bin, uint32_t w, uint32_t h) {
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

void System::loadMap(std::string name) {
    resetMap();

    // Get image info
    res::Image* mapImg = res::get<res::Image>("maps/" + name + ".png");
    uint32_t w = mapImg->getWidth();
    uint32_t h = mapImg->getHeight();
    uint32_t c = mapImg->getChannels();
    uint8_t* data = mapImg->getData();

    // Update map component
    MapComponent* mapComponent = map.get<MapComponent>();
    for (int i = 0, j = 0; i < w * h * c; i += c, j++)
        mapComponent->grid[j] = data[i] > 127 ? false : true;
    std::vector<Box> boxes = parseBoxes(mapComponent->grid, w, h);

    // Create parsed boxes
    for (const Box& box : boxes) {
        cmp::Entity wall = cmp::createEntity();

        atta::vec2 pos = atta::vec2(box.pos.x, box.pos.y) * MapComponent::cellSize;
        atta::vec2 size = atta::vec2(box.size.x, box.size.y) * MapComponent::cellSize;
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

    // Generate collision grid
    generateCollisionGrid();

    LOG_SUCCESS("System", "Map [w]$0[] loaded successfully", name);
}

void System::resetMap() {
    for (cmp::Entity child : walls.getChildren())
        cmp::destroyEntity(child);
}

void System::generateCollisionGrid() {
    MapComponent* mapComponent = map.get<MapComponent>();
    MapComponent::Grid& g = mapComponent->grid;
    MapComponent::Grid& c = mapComponent->collisionGrid;

    for (int y = 0; y < MapComponent::height; y++) {
        for (int x = 0; x < MapComponent::width; x++) {
            bool isColliding = false;
            if (x <= 1 || y <= 1 || x >= MapComponent::width - 2 || y >= MapComponent::height - 2) {
                isColliding = true;
            } else {
                for (int i = -2; i < 2; i++)
                    for (int j = -2; j < 2; j++)
                        if (g[(y + j) * MapComponent::height + (x + i)])
                            isColliding = true;
            }
            c[y * MapComponent::height + x] = isColliding;
        }
    }
}
