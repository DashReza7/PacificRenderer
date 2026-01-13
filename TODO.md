# TODO

## Notes

- The scene parser can't parse all mitsuba3 files.

## Important

- bdpt mis weight calculation when t can be 1 is biased. (for $t\geq2$ is correct).
- bdpt s=0 strategy. Also its performance is bad (should implement efficiency-aware RR).
- fix non-symmetric behaviour in dielectric, shading normals
- bug in .hdr files
- fix ray triangle(or BVH) bug having problem with large or small Epsilon(in the dragon scene)
- add adaptive position based cosine weighted sampling
- possible bug in BVH build (AABB of flat surfaces might be too thin)
- bug in Beckmann distribution. sometimes returning nan

## Others

- implement bilinear texture interpolation
- implement Volumetric integrator
- compute per-vertex normals when not provided
- implement a Transform class, and apply all transformations with it.
- implement ray differential for texture sampling
- create a UI for setting up the scene and changing parameters (maybe with nanogui)
- organize the codebase into namespaces, like the math utils.
- Error handling in SceneParser
- by default use manual. additionally provide the opportunity to use Embree or Optix
- make bvh build parallel
- implement normal & bump mapping
- implement Disney principled BRDF/BSDF
- Fix the names to be more consistent (e.g. Scene vs scene, BSDF vs bsdf, etc.)
- check to delete all allocated memory
- Checkerboard texture doesn't match with mitsuba when scaled (Sphere)
- add IMath and OpenJPH as submodules
- implement scene file validity verification
- check for the correctness of cosine term multiplied in specular bsdfss
- inconsistency in hdr envmap values
- film and sampler may be defined outisde sensor and referenced via id
- add wirefrarme integrator
