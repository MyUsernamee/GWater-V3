#pragma once

#include <NvFlex.h>
#include <NvFlexExt.h>
#include <NvFlexDevice.h>
#include <vector>
#include "GarrysMod/Lua/Interface.h"
#include <string>
#include <map>

//Float4 structure, holds 4 floats, X, Y, Z, and W
struct float4 {
    float x, y, z, w;
    float4(float x1, float y1, float z1, float w1) : x(x1), y(y1), z(z1), w(w1) {};
    float4() : x(0), y(0), z(0), w(0) {};
    float4(float l) : x(l), y(l), z(l), w(l) {};
    float4(Vector l, float w1) : x(l.x), y(l.y), z(l.z), w(w1) {};
    float4 operator+(float4 other) {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
    }
};

//Float3 structure, holds 3 floats, X, Y, and Z
struct float3 {
    float x, y, z;
    float3(float x1, float y1, float z1) : x(x1), y(y1), z(z1) {};
    float3() : x(0), y(0), z(0) {};
    float3(float l) : x(l), y(l), z(l) {};
    float3(Vector l) : x(l.x), y(l.y), z(l.z) {};
    float3(float4 l) : x(l.x), y(l.y), z(l.z) {};
    float3 operator+(float3 e) {
        return { x + e.x, y + e.y, z + e.z };
    }
    float3 operator*(float3 e) {
        return { x * e.x, y * e.y, z * e.z };
    }
    float3 operator-(float3 e) {
        return { x - e.x, y - e.y, z - e.z };
    }
    float3 operator/(float e) {
        return { x / e, y / e, z / e };
    }
    float3 Cross(float3 e) {
        return { y * e.z - z * e.y, z * e.x - x * e.z, x * e.y - y * e.x };
    }
    float3 Normalize() {
        float len = sqrt(x * x + y * y + z * z);
        return { x / len, y / len, z / len };
    }
};

//Mat3 structure, holds 3 float3s
//Mat3 structure, holds 3 float3s
struct mat3
{
    float3 column1;
    float3 column2;
    float3 column3;

    mat3(float3 c1, float3 c2, float3 c3) : column1(c1), column2(c2), column3(c3) {};
    mat3() : column1(float3(0, 0, 0)), column2(float3(0, 0, 0)), column3(float3(0, 0, 0)) {};
    mat3(float l) : column1(float3(l, l, l)), column2(float3(l, l, l)), column3(float3(l, l, l)) {};
    mat3(float3 lookingDirection){

        float3 up = float3(0, 0, 1);
        float3 xaxis = lookingDirection.Cross(up);
        float3 yaxis = xaxis.Cross(lookingDirection);

        xaxis = xaxis.Normalize();
        yaxis = yaxis.Normalize();

        column1.x = lookingDirection.x; // This should be 1
        column1.y = xaxis.x; // This should be 0
        column1.z = yaxis.x; // This should be 0

        column2.x = lookingDirection.y; // This should be 0
        column2.y = xaxis.y; // This should be 1
        column2.z = yaxis.y; // This should be 0

        column3.x = lookingDirection.z; // This should be 0
        column3.y = xaxis.z; // This should be 0
        column3.z = yaxis.z; // This should be 1

    }

    void ConstructLocalSpace(float3 xaxis, float3 yaxis, float3 lookingDirection){

        column1.x = lookingDirection.x; // This should be 1
        column1.y = xaxis.x; // This should be 0
        column1.z = yaxis.x; // This should be 0

        column2.x = lookingDirection.y; // This should be 0
        column2.y = xaxis.y; // This should be 1
        column2.z = yaxis.y; // This should be 0

        column3.x = lookingDirection.z; // This should be 0
        column3.y = xaxis.z; // This should be 0
        column3.z = yaxis.z; // This should be 1

    }

    void Transpose() {
        column1 = float3(column1.x, column2.x, column3.x);
        column2 = float3(column1.y, column2.y, column3.y);
        column3 = float3(column1.z, column2.z, column3.z);
    }

    float3 operator*(float3 e) {
        return { column1.x * e.x + column2.x * e.y + column3.x * e.z,
                 column1.y * e.x + column2.y * e.y + column3.y * e.z,
                 column1.z * e.x + column2.z * e.y + column3.z * e.z };
    }

    
};

struct Particle {
    float4 pos;
    float3 vel;
};

struct Prop {
    NvFlexBuffer* verts;
    NvFlexBuffer* indices;
    int meshID;
    
    float4 pos;
    float4 lastPos;

    float4 ang;
    float4 lastAng;
};

struct SimBuffers {
    float4* particles;
    float3* velocities;
    int* phases;
    int* activeIndices;
    NvFlexCollisionGeometry* geometry;
    float4* positions;
    float4* rotations;
    float4* prevPositions;
    float4* prevRotations;
    int* flags;
    int* indices;
    float* lengths;
    float* coefficients;
};

struct ForceFieldData {
    NvFlexExtForceFieldCallback* forceFieldCallback;
    NvFlexExtForceField* forceFieldBuffer;
    int forceFieldCount;
};


class FLEX_API {
    NvFlexLibrary* flexLibrary;
    NvFlexSolver* flexSolver;
    SimBuffers* simBuffers;

    NvFlexBuffer* particleBuffer;
    NvFlexBuffer* velocityBuffer;
    NvFlexBuffer* phaseBuffer;
    NvFlexBuffer* activeBuffer;

    NvFlexBuffer* geometryBuffer;
    NvFlexBuffer* geoFlagsBuffer;
    NvFlexBuffer* geoPosBuffer;
    NvFlexBuffer* geoQuatBuffer;
    NvFlexBuffer* geoPrevPosBuffer;
    NvFlexBuffer* geoPrevQuatBuffer;

    NvFlexBuffer* indicesBuffer;
    NvFlexBuffer* lengthsBuffer;
    NvFlexBuffer* coefficientsBuffer;

    ForceFieldData* forceFieldData;

    std::vector<Prop> props;
    std::vector<Particle> particleQueue;

public:
    std::map<std::string, float*> flexMap;
    std::map<std::string, float> gwaterMap;
    NvFlexParams* flexParams;
    NvFlexSolverDesc flexSolverDesc;

    float radius;

    void addParticle(Vector pos, Vector vel);
    void addMeshConcave(GarrysMod::Lua::ILuaBase* LUA);
    void addMeshConvex(GarrysMod::Lua::ILuaBase* LUA);
    void updateMeshPos(Vector pos, QAngle ang, int id);
    void freeProp(int ID);

    void updateParam(std::string, float n);
    void updateExtraParam(std::string str, float n);
    void initParams();
    void initParamsRadius(float r);
    void flexSolveThread();
    void removeAllParticles();
    void removeAllProps();
    int removeInRadius(float3 pos, float radius);

    void cullParticles();
    void cleanLostParticles();
    int cleanLoneParticles();

    void applyForce(float3 pos, float3 vel, float radius, bool linear);
    void applyForceOutwards(float3 pos, float strength, float radius, bool linear);

    //void addCloth(GarrysMod::Lua::ILuaBase* LUA, size_t tableLen);

    void addForceField(Vector pos, float radius, float strength, bool linear, int type);
    void deleteForceField(int ID);
    void setForceFieldPos(int ID, Vector pos);
    void editForceField(int ID, float radius, float strength, bool linear, int type);

    void mapBuffers();
    void unmapBuffers();
    FLEX_API();
    ~FLEX_API();
};

