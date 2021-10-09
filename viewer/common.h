#pragma once
#include <vector>
#include <string>
enum MeshMode
{
    MESH_MODE_PLAIN,
    MESH_MODE_SKINNING_COLORED,
    MESH_MODE_TEXTURE,
    NB_MESH_MODES
};

static std::vector<std::string> meshmode_names = {"MESH_MODE_PLAIN",
                                                  "MESH_MODE_SKINNING_COLORED",
                                                  "MESH_MODE_TEXTURE"};

enum OperationMode
{
    // Object Mode: change the whole subtoy's shape or position
    OPMODE_OBJECT_MODE,
    OPMODE_DRAW,
    OPMODE_DRAW_SYM,
    OPMODE_SELECT_TOY,
    OPMODE_SELECT_PARENT_TOY,
    OPMODE_MOVE,
    OPMODE_ROTATE,
    OPMODE_SCALE,

    // Painting
    OPMODE_GET_COLOR,
    OPMODE_COLOR_FILL,
    OPMODE_TEXTURE_PAINT,
    OPMODE_TEXPAINT_IMAGE,

    // Edit Mode: change vertex position or bone position
    OPMODE_EDIT_MODE,
    OPMODE_MODIFY_SKETCH_LINES,
    OPMODE_EDIT_BONE,
    OPMODE_EDIT_POINT_HANDLE,

    // Pose Mode: used for animation, does not affect the original mesh
    OPMODE_POSE_MODE,
    OPMODE_BBW, // linear method, compute skinning weights
    OPMODE_BBW_DEFORM,
    // OPMODE_ARAP_DEFORM_3D,
    // OPMODE_ARAP_DEFORM_2D, //Non-linear method
};

inline OperationMode getGlobalMode(OperationMode subOpMode)
{
    switch (subOpMode)
    {
    case OPMODE_DRAW:
    case OPMODE_DRAW_SYM:
    case OPMODE_SELECT_TOY:
    case OPMODE_SELECT_PARENT_TOY:
    case OPMODE_MOVE:
    case OPMODE_ROTATE:
    case OPMODE_SCALE:
        return OPMODE_OBJECT_MODE;

    case OPMODE_MODIFY_SKETCH_LINES:
    case OPMODE_EDIT_BONE:
    case OPMODE_EDIT_POINT_HANDLE:
        return OPMODE_EDIT_MODE;

    case OPMODE_BBW:
    case OPMODE_BBW_DEFORM:
        return OPMODE_POSE_MODE;
    default:
        return OPMODE_OBJECT_MODE;
    }
}

static std::vector<std::string> opmode_names = {
    "OPMODE_OBJECT_MODE",
    "OPMODE_DRAW",
    "OPMODE_DRAW_SYM",
    "OPMODE_SELECT_TOY",
    "OPMODE_SELECT_PARENT_TOY",
    "OPMODE_MOVE",
    "OPMODE_ROTATE",
    "OPMODE_SCALE",

    // Painting
    "OPMODE_GET_COLOR",
    "OPMODE_COLOR_FILL",
    "OPMODE_TEXTURE_PAINT",
    "OPMODE_TEXPAINT_IMAGE",

    // Edit Mode: change vertex position or bone position
    "OPMODE_EDIT_MODE",
    "OPMODE_MODIFY_SKETCH_LINES",
    "OPMODE_EDIT_BONE",
    "OPMODE_EDIT_POINT_HANDLE",

    // Pose Mode: used for animation, does not affect the original mesh
    "OPMODE_POSE_MODE",
    "OPMODE_BBW", // linear method, compute skinning weights
    "OPMODE_BBW_DEFORM"};

enum ShadeProgramID
{
    PROGRAM_POINT,
    PROGRAM_LINE,
    PROGRAM_MESH,
    PROGRAM_MESH_WITH_TEXTURE,
    PROGRAM_BG,
    PROGRAM_AXIS,
    NB_OF_PROGRAMS
};

#ifndef Q_DEBUG
#define Q_DEBUG qDebug() << __FILE__ << " " << __LINE__ << " "
#endif