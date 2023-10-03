#include "AMPCore.h"
#include "../link/Link.h"

class Manipulator {
    public:
        // constructors:
        Manipulator();
        ~Manipulator();

        // methods:
        void hereIsAMethod();

        // fields:
        std::vector<Link> links;
};