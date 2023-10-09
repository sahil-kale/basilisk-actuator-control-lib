#ifndef BRIDGE_HBRIDGE_HPP
#define BRIDGE_HBRIDGE_HPP

namespace hwbridge {

// Define a generic h-bridge class
class HBridge {
   public:
    HBridge() = default;
    virtual ~HBridge() = default;

    // Define a virtual function to run the h-bridge
    virtual void run(float speed) = 0;
};

}  // namespace hwbridge

#endif