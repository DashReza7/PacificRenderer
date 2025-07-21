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
 - add filename to Shape
 - Implement a logger with several log-levels
 - Test all MathUtils functions
 - change Scene parsing file names to SceneDesc (and others too) to avoid confusion with the main Scene (& Integrator, etc.) classes
 - seperate implementations from header files to source files
 - setup clang-llvm as the compiler
 - progress bar
 - learn how to resolve conflicts (like assimp and zlib)
 - by default use manual. additionally provide the opportunity to use Embree or Optix
 - use GLM instead of your customly defined vec/matrix types

