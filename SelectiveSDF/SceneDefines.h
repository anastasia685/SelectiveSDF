#pragma once
#include "RayTracingHlslCompat.h"
#include "BufferHelper.hpp"

namespace GlobalRootSignature {
    namespace Slot {
        enum Enum {
            OutputView = 0,
            AccelerationStructure,
            SceneConstant,
            VertexBuffers,
            Count
        };
    }
}

namespace LocalRootSignature {
    namespace Type {
        enum Enum {
            //Triangle = 0,
            AABB = 0,
            Count
        };
    }
}
//namespace LocalRootSignature {
//    namespace Triangle {
//        namespace Slot {
//            enum Enum {
//                Count = 0
//            };
//        }
//        struct RootArguments {};
//    }
//}

namespace LocalRootSignature {
    namespace AABB {
        namespace Slot {
            enum Enum {
                ObjectConstant = 0,
                Count
            };
        }
        struct RootArguments {
            ConstantBufferTypes::ObjectConstantBuffer objectCb;
        };
    }
}

namespace LocalRootSignature {
    inline UINT MaxRootArgumentsSize()
    {
        //return max(sizeof(Triangle::RootArguments), sizeof(AABB::RootArguments));
        return sizeof(AABB::RootArguments);
    }
}

namespace GeometryType {
    enum Enum {
        Triangle = 0,
        AABB,       // Procedural geometry with an application provided AABB.
        Count
    };
}

namespace BottomLevelASType = GeometryType;

// Ray types traced in this sample.
namespace RayType {
    enum Enum {
        Radiance = 0,   // ~ Primary, reflected camera/view rays calculating color for each hit.
        //Shadow,         // ~ Shadow/visibility rays, only testing for occlusion
        Count
    };
}

namespace IntersectionShaderType = SDFPrimitive;