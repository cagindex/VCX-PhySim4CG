#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseSpinningCube.h"
#include "Labs/1-RigidBody/CaseTwoCollision.h"
#include "Labs/1-RigidBody/CaseMultiCollision.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        std::size_t _caseId = 0;

        CaseSpinningCube   _caseSpinningCube;
        CaseTwoCollision   _caseTwoCollision;
        CaseMultiCollision _caseMultiCollision;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _caseSpinningCube,
            _caseTwoCollision,
            _caseMultiCollision,
        };

    public:
        App();

        void OnFrame() override;
    };
}