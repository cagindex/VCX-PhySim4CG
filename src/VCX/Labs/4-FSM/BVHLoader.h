#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Labs/4-FSM/Action.h"
#include "Labs/4-FSM/Skeleton.h"

namespace VCX::Labs::FSM
{

    class BVHLoader
    {
    public:
        BVHLoader();

        void Load(const char* fp, Skeleton & skeleton, Action & action);

    private:

        std::vector<std::string> split(std::string& str);
        void GetLine(std::ifstream& infile, std::vector<std::string>& item);

        void ConstructTree(Joint * & ptr, std::string const& Name, std::ifstream& infile);
        void ConstructAction(Action & action, std::ifstream & infile);

        std::string     EndSiteName = "???";
    };
}