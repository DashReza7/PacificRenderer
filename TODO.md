**Notes:**
 - The scene parser can't parse all mitsuba3 files.
 - The BSDF referencing doesn't support forward referencing.

**TODO:**
 - Convert all () functions with \{\} 
 - organize the	codebase into namespaces, like the math utils.
 - look for all float and change them to Float (SceneParser.h is suspiciosu)
 - Error handling in SceneParser
 - make sceneParser scene printer prettier (convert to json, like in mitsuba3)
 - adjust the formatting in sceneParser (camel case and others)
 - Implement a logger with several log-levels
 - Test all MathUtils functions
 - seperate implementations from header files to source files
 - progress bar
 - by default use manual. additionally provide the opportunity to use Embree or Optix
 - replace all smart pointers with regular pointers
 - add a mesh (vector of Triangle/Quad) geometry type
 - make bvh build parallel
 - implement AOV integrator
 - implement Volumetric integrator
 - implement normal & bump mapping
 - implement Disney principled BRDF/BSDF
