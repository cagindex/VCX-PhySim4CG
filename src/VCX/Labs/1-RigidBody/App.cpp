#include "Labs/1-RigidBody/App.h"
#include "Assets/bundled.h"

namespace VCX::Labs::RigidBody {

    App::App():
        _ui(Labs::Common::UIOptions {}),
        _caseSpinningCube(),
        _caseTwoCollision(),
        _caseMultiCollision()
             {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
