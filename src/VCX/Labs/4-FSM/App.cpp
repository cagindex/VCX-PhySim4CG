#include "Assets/bundled.h"
#include "Labs/4-FSM/App.h"

namespace VCX::Labs::FSM {

    App::App():
        _ui(Labs::Common::UIOptions {}),
        _caseHang(),
        _caseDrop(),
        _case3DTest() {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
