#include "node.hpp"
#include <bit>
#include <fstream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Communication_node");
    Communication bringup;



    // if constexpr (std::endian::native == std::endian::big)
    //     std::cout << "big-endian\n";
    // else if constexpr (std::endian::native == std::endian::little)
    //     std::cout << "little-endian\n";
    // else
    //     std::cout << "mixed-endian\n";
    
}
