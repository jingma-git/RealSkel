#pragma once

#include <map>
#include <vector>
#include <Eigen/Core>
class Bone;
// Map that maps bone pointers to bone "copies" or objects. To be used in
// animations.
class BoneBoneCopyMap : public std::map<Bone *, Bone>
{
    // Constructor
public:
    // Construct a map for each bone found in the given bone forest (list of
    // boen roots) use pointers to these bones as keys and copies as values
    // Input:
    //   BR  list of bone roots
    //   rest  whether or not to use rest pose values
    BoneBoneCopyMap(std::vector<Bone *> BR, bool rest);
};

// Abreviated iterator
typedef BoneBoneCopyMap::iterator BBCMIterator;
