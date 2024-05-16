#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/3-FEM/CaseTest.h"
#include "Labs/3-FEM/CaseCube.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::FEM {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseTest      _caseTest;
        CaseCube      _caseCube;
        
        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseTest, _caseCube };

    public:
        App();

        void OnFrame() override;
    };
}
