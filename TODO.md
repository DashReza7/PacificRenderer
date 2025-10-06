**Notes:**
 - The scene parser can't parse all mitsuba3 files.
 - The BSDF referencing doesn't support forward referencing.

**TODO:**

Urgent:
 - Implement two-sided BSDFs
 - Implement a logger with several log-levels

Others:
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
 