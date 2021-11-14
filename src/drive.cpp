#include "drive.h"
#include <algorithm>

namespace driving
{
	std::unordered_map<int, int> genDrivingLut() {
              std::unordered_map<int, int> umap;
              for(int i = -127; i <= 127; i++) {
                        umap[i] = expDrive(cvals(i));
              }
              return umap;
      }
}
