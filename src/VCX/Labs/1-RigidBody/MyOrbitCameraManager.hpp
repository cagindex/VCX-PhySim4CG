#pragma once

#include "Labs/Common/OrbitCameraManager.h"

using OrbitCamera = VCX::Labs::Common::OrbitCameraManager;

namespace VCX::Labs::RigidBody{
    class MyOrbitCameraManager : public OrbitCamera {
    public:
        MyOrbitCameraManager():OrbitCamera(){}

        void ProcessInput(Engine::Camera & camera, ImVec2 const & mousePos) {
            auto            window  = ImGui::GetCurrentWindow();
            ImGuiIO const & io      = ImGui::GetIO();
            bool            anyHeld = false;
            bool            hover   = false;
            ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hover, &anyHeld);
            bool         leftHeld  = anyHeld && ImGui::IsMouseDown(ImGuiMouseButton_Left);
            bool         rightHeld = anyHeld && ImGui::IsMouseDown(ImGuiMouseButton_Right);
            ImVec2 const delta     = io.MouseDelta;
            float        wheel     = io.MouseWheel;
            bool         wheeling  = EnableZoom && wheel != 0.f && hover;
            bool         moving    = (delta.x != 0.f || delta.y != 0.f) && hover;
            bool         funcKey   = io.KeyCtrl || io.KeyShift;
            bool         altKey    = io.KeyAlt;

            float heightNorm     = 1.f / window->Rect().GetHeight();
            bool  rotating       = EnableRotate && moving && ((leftHeld) || (rightHeld));
            bool  panningByMouse = EnablePan && moving && ((funcKey && leftHeld) || (! funcKey && rightHeld));

            bool movingByMouse = moving && altKey && leftHeld;

            bool panningByKeyBoard[6] = {
                ImGui::IsItemFocused() && ImGui::IsKeyDown(ImGuiKey_UpArrow),
                ImGui::IsItemFocused() && ImGui::IsKeyDown(ImGuiKey_DownArrow),
                ImGui::IsItemFocused() && ImGui::IsKeyDown(ImGuiKey_LeftArrow),
                ImGui::IsItemFocused() && ImGui::IsKeyDown(ImGuiKey_RightArrow),
                ImGui::IsItemFocused() && ImGui::IsKeyDown(ImGuiKey_Q),
                ImGui::IsItemFocused() && ImGui::IsKeyDown(ImGuiKey_E)
            };

            if (ImGui::IsItemFocused() && ImGui::IsKeyPressed(ImGuiKey_R)) {
                Reset(camera);
                return;
            }

            _state = StateNone;
            if (anyHeld) {
                _state |= StateHold;
            }
            if (movingByMouse || panningByMouse || (EnablePan && (panningByKeyBoard[0] || panningByKeyBoard[1] || panningByKeyBoard[2] || panningByKeyBoard[3] || panningByKeyBoard[4] || panningByKeyBoard[5]))) {
                // perspective
                glm::vec3 direction      = camera.Target - camera.Eye;
                float     targetDistance = glm::length(direction);
                glm::quat q              = glm::quatLookAt(direction / targetDistance, camera.Up);
                // half of the fov is center to top of screen
                targetDistance *= glm::tan(glm::radians(camera.Fovy) * 0.5f);
                float panX = (panningByMouse || movingByMouse) ? -delta.x * heightNorm : 0.f;
                float panY = (panningByMouse || movingByMouse) ? delta.y * heightNorm : 0.f;
                float panZ = 0.f;
                if (panningByKeyBoard[0]) panZ -= 0.02f;
                if (panningByKeyBoard[1]) panZ += 0.02f;
                if (panningByKeyBoard[2]) panX -= 0.02f;
                if (panningByKeyBoard[3]) panX += 0.02f;
                if (panningByKeyBoard[4]) panY -= 0.02f;
                if (panningByKeyBoard[5]) panY += 0.02f;
                // we use only clientHeight here so aspect ratio does not distort speed
                float     panLeft  = 2 * panX * PanSpeed * targetDistance;
                float     panUp    = 2 * panY * PanSpeed * targetDistance;
                float     panFront = 2 * panZ * PanSpeed * targetDistance;
                glm::vec3 front    = glm::cross(q * glm::vec3(1, 0, 0), camera.Up);
                if (movingByMouse) {
                    if (ScreenSpacePanning) {
                        _moveDelta -= q * glm::vec3(panLeft, panUp, 0.f) + panFront * front;
                    } else {
                        _moveDelta -= q * glm::vec3(panLeft, 0.f, 0.f) + panUp * camera.Up + panFront * front;
                    }
                    _state |= StateMove;
                } else {
                    if (ScreenSpacePanning) {
                        _panOffset += q * glm::vec3(panLeft, panUp, 0.f) + panFront * front;
                    } else {
                        _panOffset += q * glm::vec3(panLeft, 0.f, 0.f) + panUp * camera.Up + panFront * front;
                    }
                    _state |= StatePan;
                }
            }
            if (! altKey) {
                if (rotating) {
                    _spDelta.Theta -= delta.x * RotateSpeed * heightNorm * (glm::pi<float>() * 2);
                    _spDelta.Phi -= delta.y * RotateSpeed * heightNorm * (glm::pi<float>() * 2);
                    _state |= StateRotate;
                }
                if (wheeling) {
                    _logScale += ZoomSpeed * wheel;
                    _state |= StateDolly;
                }
            }
        }
    };
}