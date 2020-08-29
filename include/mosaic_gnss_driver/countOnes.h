#ifndef COUNTONES_H
#define COUNTONES_H

/// Count the number of 1s in binary representation of a number.
///
/// Constructor takes in an integer as argument, getCount method returns the number of 1s in it's binary representation
class CountOnes
{
private:
    int m_count;

public:
    CountOnes(int n);
    int getCount();
};

#endif