#include "Assets/bundled.h"
#include "Labs/3-FEM/App.h"

namespace VCX::Labs::FEM {

    App::App():
        _ui(Labs::Common::UIOptions {}),
        _caseTest(),
        _caseCube() {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
