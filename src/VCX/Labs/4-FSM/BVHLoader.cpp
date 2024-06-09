#define GLM_ENABLE_EXPERIMENTAL
#include "Labs/4-FSM/BVHLoader.h"

namespace VCX::Labs::FSM
{
    BVHLoader::BVHLoader(){}

    void BVHLoader::Load(const char* fp, Skeleton & skeleton, Action & action)
    {
        std::string str;

        std::ifstream infile;
        infile.open(fp);

        bool Hierachy = true;

        while(std::getline(infile, str))
        {
            auto item = split(str);
            if (item.size() == 0) continue; // avoid space line

            if (item.at(0) == "HIERARCHY") Hierachy = true;
            if (item.at(0) == "MOTION")    Hierachy = false;

            if (Hierachy)
            {
                const std::string & tmp = item.at(0);
                if (tmp == "ROOT")
                {
                    ConstructTree(skeleton.Root, item.at(1), infile);
                }
            }
            else
            {
                ConstructAction(action, infile);
            }
        }

        infile.close();
    }


    void BVHLoader::ConstructTree(Joint * & ptr, std::string const& Name, std::ifstream& infile)
    {
        std::vector<std::string> item;

        GetLine(infile, item); // get '{'

        ptr       = new Joint();
        ptr->Name = Name;

        // Get Offset
        GetLine(infile, item);
        ptr->LocalOffset = { std::stof(item.at(1)), std::stof(item.at(2)), std::stof(item.at(3)) };

        if (Name != EndSiteName)
        {
            // Get Channels
            GetLine(infile, item);
            for (int i = 0; i < std::stoi(item.at(1)); ++i)
            {
                std::string const& tmp = item.at(2+i);
                if      (tmp == "Xrotation")
                    ptr->RotationIdx[i%3] = 0;
                else if (tmp == "Yrotation")
                    ptr->RotationIdx[i%3] = 1;
                else if (tmp == "Zrotation")
                    ptr->RotationIdx[i%3] = 2;
                else if (tmp == "Xposition")
                    ptr->PositionIdx[i%3] = 0;
                else if (tmp == "Yposition")
                    ptr->PositionIdx[i%3] = 1;
                else if (tmp == "Zposition")
                    ptr->PositionIdx[i%3] = 2;
                else
                    std::cerr << "UnKnow Character encounterd while reading bvh: " << tmp << std::endl; 
            }
        }

        Joint *NextPtr = nullptr;
        while(GetLine(infile, item), item.at(0) != "}")
        {
            std::string tmp;
            if (item.at(0) == "JOINT") tmp = item.at(1);
            else                       tmp = EndSiteName;

            if (NextPtr == nullptr)
            {
                ConstructTree(ptr->ChiPtr, tmp, infile);
                NextPtr = ptr->ChiPtr;
            }
            else
            {
                ConstructTree(NextPtr->BroPtr, tmp, infile);
                NextPtr = NextPtr->BroPtr;
            }
        }
    }


    void BVHLoader::ConstructAction(Action & action, std::ifstream & infile)
    {
        std::vector<std::string> item;

        // Get Frames
        GetLine(infile, item);
        if (item.at(0) == "Frames:") action.Frames = std::uint32_t( std::stoi(item.at(1)) );
        else                        std::cerr << "Incomplete file struct encountered, \'Frames:\' not founded" << std::endl; 

        // Get FrameTime
        GetLine(infile, item);
        if ((item.at(0) == "Frame") && 
        (item.at(1) == "Time:")) action.FrameTime = std::stof(item.at(2));
        else                        std::cerr << "Incomplete file struct encountered, \'Frame Time:\' not founded" << std::endl; 

        // Read Whole Contains
        for (int lineNum = 0; lineNum < action.Frames; ++lineNum)
        {
            action.FrameParams.push_back(std::vector<float>());
            GetLine(infile, item);

            for (auto value : item)
            {
                action.FrameParams.at(lineNum).push_back(std::stof(value));
            }
        }
    }


    void BVHLoader::GetLine(std::ifstream& infile, std::vector<std::string>& item)
    {
        item.clear();
        std::string str;

        std::getline(infile, str);
        item = split(str);
    }

    std::vector<std::string> BVHLoader::split(std::string& str)
    {
        std::vector<std::string> ret_value;

        std::uint32_t StartIdx = 0, EndIdx = 0;

        // Init
        while(( str.at(StartIdx) == '\t' ) || ( str.at(StartIdx) == ' ')) 
        {
            StartIdx ++;
            if (str.length() == StartIdx) return ret_value;
        }
        EndIdx = StartIdx;


        str = str + " ";
        while(EndIdx != str.length())
        {
            if (( str.at(EndIdx) == '\t' ) || ( str.at(EndIdx) == ' ' ))
            {
                ret_value.push_back(str.substr(StartIdx, EndIdx - StartIdx));
                // Renew StartIdx to next nonspace char
                StartIdx = (++EndIdx);
            }
            else
            {
                EndIdx ++;
            }
        }

        return ret_value;
    }
}