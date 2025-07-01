#pragma once

using namespace DirectX;
using namespace std;

struct BoundingBox
{
    XMFLOAT3 min;
    XMFLOAT3 max;
};
struct InstanceData
{
    UINT index;
    BoundingBox boundingBox;
};
struct BVHNode {
    BoundingBox boundingBox;
    int leftChild = -1;
    int rightChild = -1;
    int firstInstance = -1;
    int instanceCount = 0;
};

class BVHBuilder
{
public:
    vector<BVHNode> m_nodes;
    vector<UINT> m_flatInstanceIndices;

    void Build(const vector<ObjectInstance>& instances, UINT offset, size_t maxLeafSize = 4) 
    {
        UINT count = instances.size() - offset;
        m_instances.resize(count);

        vector<UINT> indices(static_cast<UINT>(count));
        for (UINT i = 0; i < count; ++i)
        {
            m_instances[i] = { i + offset, computeWorldAABB(instances[i + offset]) };
            indices[i] = i;
        }

        buildRecursive(indices, maxLeafSize);
    }

private:
    vector<InstanceData> m_instances;

    int buildRecursive(const vector<UINT>& instanceIndices, size_t maxLeafSize)
    {
        BVHNode node;
        computeBounds(instanceIndices, node.boundingBox.min, node.boundingBox.max);

        if (instanceIndices.size() <= maxLeafSize) {
            // Leaf node
            node.firstInstance = static_cast<UINT>(m_flatInstanceIndices.size());
            node.instanceCount = static_cast<UINT>(instanceIndices.size());

            m_flatInstanceIndices.insert(m_flatInstanceIndices.end(), instanceIndices.begin(), instanceIndices.end());

            int index = static_cast<int>(m_nodes.size());
            m_nodes.push_back(node);
            return index;
        }

        // Internal node — split along the longest axis of bounding box
        float extentX = node.boundingBox.max.x - node.boundingBox.min.x;
        float extentY = node.boundingBox.max.y - node.boundingBox.min.y;
        float extentZ = node.boundingBox.max.z - node.boundingBox.min.z;

        int axis = 0;
        if (extentY > extentX) axis = 1;
        if (extentZ > ((axis == 0) ? extentX : extentY)) axis = 2;

        auto sorted = instanceIndices;
        std::sort(sorted.begin(), sorted.end(), [&](UINT a, UINT b) {
            const BoundingBox& boxA = m_instances[a].boundingBox;
            const BoundingBox& boxB = m_instances[b].boundingBox;

            float centerA = 0.5f * (getComponent(boxA.min, axis) + getComponent(boxA.max, axis));
            float centerB = 0.5f * (getComponent(boxB.min, axis) + getComponent(boxB.max, axis));
            return centerA < centerB;
            });

        size_t mid = sorted.size() / 2;
        vector<UINT> left(sorted.begin(), sorted.begin() + mid);
        vector<UINT> right(sorted.begin() + mid, sorted.end());

        int nodeIndex = static_cast<int>(m_nodes.size());
        m_nodes.emplace_back(); // placeholder for now

        int leftIdx = buildRecursive(left, maxLeafSize);
        int rightIdx = buildRecursive(right, maxLeafSize);

        node.leftChild = leftIdx;
        node.rightChild = rightIdx;

        m_nodes[nodeIndex] = node;
        return nodeIndex;
    }
    void computeBounds(const vector<UINT>& instanceIndices, XMFLOAT3& outMin, XMFLOAT3& outMax) {
        assert(!instanceIndices.empty());

        const auto& first = m_instances.at(instanceIndices[0]);
        XMVECTOR minVec = XMLoadFloat3(&first.boundingBox.min);
        XMVECTOR maxVec = XMLoadFloat3(&first.boundingBox.max);

        for (size_t i = 1; i < instanceIndices.size(); ++i) {
            const auto& inst = m_instances.at(instanceIndices[i]);
            XMVECTOR instMin = XMLoadFloat3(&inst.boundingBox.min);
            XMVECTOR instMax = XMLoadFloat3(&inst.boundingBox.max);

            minVec = XMVectorMin(minVec, instMin);
            maxVec = XMVectorMax(maxVec, instMax);
        }

        XMStoreFloat3(&outMin, minVec);
        XMStoreFloat3(&outMax, maxVec);
    }

    BoundingBox computeWorldAABB(const ObjectInstance& instance)
    {
        float s = instance.scale;
        XMFLOAT3 halfExtent = { 0.5f * s, 0.5f * s, 0.5f * s };

        BoundingBox box;
        box.min = {
            instance.position.x - halfExtent.x,
            instance.position.y - halfExtent.y,
            instance.position.z - halfExtent.z
        };
        box.max = {
            instance.position.x + halfExtent.x,
            instance.position.y + halfExtent.y,
            instance.position.z + halfExtent.z
        };
        return box;
    }
    inline float getComponent(const XMFLOAT3& v, int axis) {
        return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
    }
};