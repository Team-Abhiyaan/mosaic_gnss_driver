#include <mosaic_gnss_driver/countOnes.h>
#include <iostream>

int main(void)
{
    for (int i = 0; i < 101; i++)
    {
        std::cout << i << " " << countOnes(i) << std::endl;
    }

    return 0;
}