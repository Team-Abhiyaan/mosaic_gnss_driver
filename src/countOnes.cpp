#include <mosaic_gnss_driver/countOnes.h>

/// Find 1 count in n
/// 
/// @param n Input integer
CountOnes::CountOnes(int n)
{
    this->m_count = 0;

    while (n != 0)
    {
        n = n & (n - 1);
        m_count++;
    }
}

int CountOnes::getCount()
{
    return this->m_count;
}