#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/4-FSM/CaseHang.h"
#include "Labs/4-FSM/CaseDrop.h"
// #include "Labs/4-FSM/Case3DTest.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::FSM {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseHang      _caseHang;
        CaseDrop      _caseDrop;
        // Case3DTest    _case3DTest;
        
        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
             _caseHang, _caseDrop };

    public:
        App();

        void OnFrame() override;
    };
}
