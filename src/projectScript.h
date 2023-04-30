//--------------------------------------------------
// Monte Carlo SLAM
// projectScript.h
// Date: 2023-30-04
//--------------------------------------------------
#ifndef PROJECT_SCRIPT_H
#define PROJECT_SCRIPT_H
#include <atta/script/projectScript.h>

namespace scr = atta::script;

class ProjectScript : public scr::ProjectScript {
  public:
    //---------- Simulation ----------//
    void onLoad() override;
    void onUnload() override;
    void onStart() override;
    void onStop() override;
    void onAttaLoop() override;

    //---------- UI ----------//
    void onUIRender() override;

  private:
    void loadMap(std::string name);
    void resetMap();
};

ATTA_REGISTER_PROJECT_SCRIPT(ProjectScript)

#endif // PROJECT_SCRIPT_H
