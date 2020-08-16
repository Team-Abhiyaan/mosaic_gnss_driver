#ifndef COUNTONES_H
#define COUNTONES_H

//! Count the number of 1s in binary representation of n.
class CountOnes
{
private:
    int m_count;

public:
    CountOnes(int n);
    int getCount();
};

#endif