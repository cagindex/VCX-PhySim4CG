#define GLM_ENABLE_EXPERIMENTAL
#include "Labs/4-FSM/Skeleton.h"

namespace VCX::Labs::FSM
{
    Joint::Joint(){}
    Skeleton::Skeleton(){}

    std::pair<std::vector<glm::vec3>, std::vector<std::uint32_t>> Skeleton::Convert() const
    {
        std::vector<glm::vec3>     Positions;
        std::vector<std::uint32_t> Indices;

        Construct(Root, Positions, Indices);

        return {Positions, Indices};
    }

    void Skeleton::Construct(const Joint *ptr, std::vector<glm::vec3> & Positions, std::vector<std::uint32_t> & Indices) const
    {
        std::uint32_t FatherIdx = Positions.size();
        Positions.push_back(ptr->GlobalPosition);

        Joint * NextPtr = ptr->ChiPtr; 
        while(NextPtr != nullptr)
        {
            std::uint32_t ChildIdx = Positions.size();
            Positions.push_back(NextPtr->GlobalPosition);
            Indices.push_back(FatherIdx);
            Indices.push_back(ChildIdx);
            Construct(NextPtr, Positions, Indices);

            NextPtr = NextPtr->BroPtr;
        }
    }

    void Skeleton::ForwardKinematics()
    {
        // Init Root
        Root->GlobalPosition = Root->LocalOffset;
        Root->GlobalRotation = Root->LocalRotation;

        // Forward Kinematics
        ItsMyGo(Root);

        Adjust(Root);
    }

    void Skeleton::ItsMyGo(Joint * ptr)
    {
        Joint* NextPtr = ptr->ChiPtr;
        while(NextPtr != nullptr)
        {
            NextPtr->GlobalRotation = ptr->GlobalRotation * NextPtr->LocalRotation;
            NextPtr->GlobalPosition = ptr->GlobalPosition + ptr->GlobalRotation * NextPtr->LocalOffset;
            ItsMyGo(NextPtr);

            NextPtr = NextPtr->BroPtr;
        }
    }

    void Skeleton::Adjust(Joint* ptr)
    {
        ptr->GlobalPosition = Scale * ptr->GlobalPosition + Offset;

        Joint* NextPtr = ptr->ChiPtr;
        while(NextPtr != nullptr)
        {
            Adjust(NextPtr);
            NextPtr = NextPtr->BroPtr;
        }
    }
}