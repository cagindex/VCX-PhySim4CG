#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/4-FSM/CaseFSM.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::FSM {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseFSM      _caseFSM;
        
        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseFSM };

    public:
        App();

        void OnFrame() override;
    };
}
