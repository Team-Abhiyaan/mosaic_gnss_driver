#include <mosaic_gnss_driver/countOnes.h>
#include <iostream>

int main(void)
{
    CountOnes *countOnes = new CountOnes(99);
    std::cout << countOnes->getCount() << std::endl;

    return 0;
}