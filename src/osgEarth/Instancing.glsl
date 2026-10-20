#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#extension GL_EXT_gpu_shader4 : enable
#extension GL_ARB_draw_instanced: enable

#pragma vp_entryPoint oe_di_setInstancePosition
#pragma vp_location   vertex_model
#pragma vp_order      0.0

uniform samplerBuffer oe_di_postex_TBO;
// Camera-distance range [min, max) in which this geometry is drawn, per instance (osg::LOD children, see
// ConvertToDrawInstanced::apply(osg::LOD&)). Default (0, FLT_MAX).
uniform vec2 oe_di_lod_range;
uniform mat4 osg_ModelViewMatrix;

// Stage-global containing object ID
uint oe_index_objectid;
vec3 vp_Normal;

void oe_di_setInstancePosition(inout vec4 VertexMODEL)
{ 
    int index = 4 * gl_InstanceID;

    vec4 m0 = texelFetch(oe_di_postex_TBO, index);
    vec4 m1 = texelFetch(oe_di_postex_TBO, index+1); 
    vec4 m2 = texelFetch(oe_di_postex_TBO, index+2); 
    vec4 m3 = texelFetch(oe_di_postex_TBO, index+3);

    // decode the ObjectID from the last column:
    
    oe_index_objectid = uint(m3[0]) + (uint(m3[1]) << 8u) + (uint(m3[2]) << 16u) + (uint(m3[3]) << 24u);
    
    // rebuild positioning matrix and transform the vert. (Note, the matrix is actually
    // transposed so we have to reverse the multiplication order.)
    mat4 xform = mat4(m0, m1, m2, vec4(0,0,0,1));

    // Per-instance LOD selection: outside its range, collapse this geometry to the instance origin.
    vec3 origin = vec3(m0.w, m1.w, m2.w);
    float range = length((osg_ModelViewMatrix * vec4(origin, 1.0)).xyz);
    if (range < oe_di_lod_range.x || range >= oe_di_lod_range.y)
    {
        VertexMODEL = vec4(origin, 1.0);
        return;
    }

    VertexMODEL = VertexMODEL * xform;

    // rotate the normal vector in the same manner.
    vp_Normal = vp_Normal * mat3(xform);
}

