// Your implementation of Fluid Simulation.
#include "assets/bundled.h"
#include "Labs/2-FluidSimulation/App.h"

int main() {
    // make linker happy
    using namespace VCX;
    return Engine::RunApp<Labs::Fluid::App>(Engine::AppContextOptions{
        .Title         = "VCX-sim Labs 1: Rigid Body",
        .WindowSize    = {1024, 768},
        .FontSize      = 16,
        .IconFileNames = Assets::DefaultIcons,
        .FontFileNames = Assets::DefaultFonts,
    });
}
