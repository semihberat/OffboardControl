#include <vector>
#include <iostream>

class CalculateCenterofGravity{
    public:
        CalculateCenterofGravity() = default;
        ~CalculateCenterofGravity() = default;

    template <typename T> T calculate_cog(std::vector<T> positions){

        size_t size_of_positions = positions.size();
        
        if (size_of_positions == 0){
            throw std::invalid_argument("Position list is empty");
        }

        T cog{};
        for(const auto& pos : positions){
            cog.lat += pos.lat;
            cog.lon += pos.lon;
        }
        cog.lat /= size_of_positions;
        cog.lon /= size_of_positions;
        return cog;
    }
};