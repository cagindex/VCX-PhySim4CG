// Your implementation of Fluid Simulation.
#include "assets/bundled.h"
#include "Labs/4-FSM/App.h"

int main() {
    // make linker happy
    using namespace VCX;
    return Engine::RunApp<Labs::FSM::App>(Engine::AppContextOptions{
        .Title         = "VCX-sim Labs 4: FSM",
        .WindowSize    = {1024, 768},
        .FontSize      = 16,
        .IconFileNames = Assets::DefaultIcons,
        .FontFileNames = Assets::DefaultFonts,
    });
}
