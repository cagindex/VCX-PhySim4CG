set_project("VCX-Labs")
set_version("2.0.0-alpha.1")
set_xmakever("2.6.9")
set_languages("cxx20")

add_rules("mode.debug", "mode.release", "mode.profile")

add_requires("fcl")
add_requires("glad")
add_requires("glfw")
add_requires("glm")
add_requires("imgui")
add_requires("spdlog")
add_requires("stb")
add_requires("fmt")
add_requires("tinyobjloader")
add_requires("yaml-cpp")
add_requires("eigen")

if is_plat("windows") then
    add_cxflags("/EHsc")
end

target("assets")
    set_kind("phony")
    set_default(true)
    after_build(function (target)
        os.mkdir(path.join(target:targetdir(), "assets"))
        os.cp("assets/*|shaders", path.join(target:targetdir(), "assets"))
        os.mkdir(path.join(target:targetdir(), "assets", "shaders"))
        os.cp("assets/shaders/*", path.join(target:targetdir(), "assets", "shaders"))
    end)
    after_install(function (target)
        os.mkdir(path.join(target:installdir(), "assets"))
        os.cp("assets/*|shaders", path.join(target:installdir(), "assets"))
        os.mkdir(path.join(target:installdir(), "assets", "shaders"))
        os.cp("assets/shaders/*", path.join(target:installdir(), "assets", "shaders"))
    end)
    after_clean(function (target)
        os.rm(path.join(target:targetdir(), "assets"))
    end)

target("engine")
    set_kind("static")
    add_packages("glad"         , { public = true })
    add_packages("glfw"         , { public = true })
    add_packages("glm"          , { public = true })
    add_packages("imgui"        , { public = true })
    add_packages("spdlog"       , { public = true })
    add_packages("stb"          , { public = true })
    add_packages("fmt"          , { public = true })
    add_packages("tinyobjloader", { public = true })
    add_packages("yaml-cpp"     , { public = true })

    add_includedirs("src/3rdparty", { public = true })
    add_includedirs("src/VCX"     , { public = true })
    add_headerfiles("src/3rdparty/**.h")
    add_headerfiles("src/3rdparty/**.hpp")
    add_files      ("src/3rdparty/**.cpp")
    add_headerfiles("src/VCX/Assets/**.h")
    add_headerfiles("src/VCX/Assets/**.hpp")
    add_headerfiles("src/VCX/Engine/**.h")
    add_headerfiles("src/VCX/Engine/**.hpp")
    add_files      ("src/VCX/Engine/**.cpp")

target("example-imgui")
    set_kind("binary")
    add_deps("engine")
    add_deps("assets")
    add_headerfiles("src/VCX/Examples/ImGui/*.h")
    add_files      ("src/VCX/Examples/ImGui/*.cpp")

target("example-triangle")
    set_kind("binary")
    add_deps("engine")
    add_deps("assets")
    add_headerfiles("src/VCX/Examples/Triangle/*.h")
    add_files      ("src/VCX/Examples/Triangle/*.cpp")

target("lab-common")
    set_kind("static")
    add_deps("engine")
    add_deps("assets")
    add_headerfiles("src/VCX/Labs/Common/*.h")
    add_files      ("src/VCX/Labs/Common/*.cpp")

target("lab-scene")
    set_kind("static")
    add_deps("engine")
    add_deps("assets")
    add_headerfiles("src/VCX/Labs/Scene/*.h")
    add_files      ("src/VCX/Labs/Scene/*.cpp")

target("lab0")
    set_kind("binary")
    add_deps("lab-common")
    add_deps("lab-scene")
    add_packages("eigen")
    add_headerfiles("src/VCX/Labs/0-GettingStarted/*.h")
    add_headerfiles("src/VCX/Labs/0-GettingStarted/*.hpp")
    add_files      ("src/VCX/Labs/0-GettingStarted/*.cpp")

target("lab1")
    set_kind("binary")
    add_deps("lab-common")
    add_packages("eigen")
    add_packages("fcl")
    add_headerfiles("src/VCX/Labs/1-RigidBody/*.h")
    add_headerfiles("src/VCX/Labs/1-RigidBody/*.hpp")
    add_files      ("src/VCX/Labs/1-RigidBody/*.cpp")

target("lab2")
    set_kind("binary")
    add_deps("lab-common")
    add_deps("lab-scene")
    add_packages("eigen")
    add_headerfiles("src/VCX/Labs/2-FluidSimulation/*.h")
    add_headerfiles("src/VCX/Labs/2-FluidSimulation/*.hpp")
    add_files      ("src/VCX/Labs/2-FluidSimulation/*.cpp")

target("lab3")
    set_kind("binary")
    add_deps("lab-common")
    add_packages("eigen")
    add_headerfiles("src/VCX/Labs/3-FEM/*.h")
    add_headerfiles("src/VCX/Labs/3-FEM/*.hpp")
    add_files      ("src/VCX/Labs/3-FEM/*.cpp")
