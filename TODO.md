**Notes:**
 - The scene parser can't parse all mitsuba3 files.
 - The BSDF referencing doesn't support forward referencing.
 - Right now, all AreaLights are one-sided.

**TODO:**

Urgent:
 - add adaptive position based cosine weighted sampling
 - implement textures
 - Implement a logger with several log-levels
 - possible bug in BVH build (AABB of flat surfaces might be too thin)
 - implement a Transform class, and apply all transformations with it.

Others:
 - create a UI for setting up the scene and changing parameters (maybe with nanogui)
 - The `hide_emitters` parameter in the scene file doesn't work right now.
 - Convert all () constructors to {}
 - organize the	codebase into namespaces, like the math utils.
 - Error handling in SceneParser
 - adjust the formatting in sceneParser (camel case and others)
 - by default use manual. additionally provide the opportunity to use Embree or Optix
 - make bvh build parallel
 - implement AOV integrator
 - implement Volumetric integrator
 - implement normal & bump mapping
 - implement Disney principled BRDF/BSDF
 - Fix the names to be more consistent (e.g. Scene vs scene, BSDF vs bsdf, etc.)
 - Add using a config file to set up small grained details, like the MIS strategy, etc.
 - check to delete all allocated memory
 - Review the to_string functions
 - Checkerboard texture doesn't match with mitsuba when scaled 
 - add IMath and OpenJPH as submodules
