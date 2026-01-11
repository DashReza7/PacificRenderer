<p align="center">
	<img src="gallery/cbox-pacific.png" alt="cbox-pacific" width="40%">
</p>

# PacificRenderer

PacificRenderer is a physically-based renderer written in C++. It aims to provide a flexible, extensible, and educational platform for photorealistic image synthesis, inspired by [Mitsuba](https://mitsuba-renderer.org/) and [PBRT](http://www.pbrt.org/). The renderer supports a subset of the Mitsuba scene file format, making it compatible with many Mitsuba scenes (with some limitations).

---

## Features

- **Physically-Based Rendering**: Implements unbiased Monte Carlo path tracing and other integrators for realistic image synthesis.
- **Mitsuba Scene Compatibility**: Supports a subset of Mitsuba's XML scene format, allowing you to use many existing Mitsuba scenes directly.
- **Modular Integrator System**: Includes the following integrators:
    - Path tracer (`path`)
	- Particle tracer (`ptracer`)
	- Unstratified path tracer (`path-unstrat`. In contrast to the regular path tracer which samples the film plane pixel by pixel(i.e. stratified), this integrator samples the film plane in an unstratified manner. It's noisier than the regular path tracer, and was written for the purpose of demonstrating the advantages of stratified sampling)
	- Bidirectional path tracer (`bidirectional`. Note that the strategies for s=0 and t=1 are not yet implemented, hence the resulting images are not better than the previous integrators)
	- Direct lighting (`direct`)
	- Depth (`depth`), Albedo (`albedo`), Geometric normal (`geometric_normal`)
- **BSDFs and Materials**:
	- Diffuse
	- Conductor (Metal)
	- Dielectric (Glass)
	- Plastic
	- Thin Dielectric
	- Rough Conductor
	- Rough Dielectric
	- (RoughPlastic and some others are not yet implemented)
- **Light Sources**:
	- Point Light
	- Area Light
	- Directional Light
	- Environment Map
- **Geometry**:
	- Triangle Meshe (**obj**, **ply**, **serialized**)
	- Sphere
	- Disk
    - Rectangle, Cube
- **Texture Support**: Bitmap, checkerboard, and constant textures.
- **Image Output**: Supports PNG, JPG, EXR, and HDR output formats.
- **Multithreading**: Parallel rendering using multiple CPU threads.
- **BVH Acceleration**: ray tracing acceleration with bounding volume hierarchy.
- **Filter Support**: Gaussian reconstruction filter.
- **Extensible Registry System**: Easily add new BSDFs, integrators, emitters, and textures.

---

## Scene File Format & Compatibility

- Scene files are written in XML and are a subset of the [Mitsuba renderer's format](https://mitsuba.readthedocs.io/en/stable/src/key_topics/scene_format.html).
- Most Mitsuba scenes are compatible, but some features are not yet implemented (e.g., RoughPlastic BSDF and Volumetric Integrator).

---

## Building & Usage

1. **Clone the repository**
2. **Build with CMake**:
	 ```sh
	 mkdir build && cd build
	 cmake ..
	 cmake --build .
	 ```
3. **Run the renderer**:
	 ```sh
	 PacificRenderer path/to/scene.xml -o output.png --progress --threads 8
	 ```
	 - `-o`/`--output_file`: Output image path
	 - `-t`/`--threads`: Number of threads to use
	 - `-p`/`--progress`: Show progress bar

---

## Limitations & TODO

- Some Mitsuba features are not yet implemented (see `TODO.md` for details):
    - Metropolis Light Transport
	- Volumetric Integrator
	- RoughPlastic BSDF
	- Disney Principled BSDF
	- UI for scene setup

---

## Gallery

Below are some images rendered with PacificRenderer. The scene files are mostly from [Mitsuba gallery](https://mitsuba.readthedocs.io/en/stable/src/gallery.html). Stanford bunny and dragon are from [Stanford 3D scanning repository](https://graphics.stanford.edu/data/3Dscanrep/).

### Cornell Box

| cornell-box | cbox-pacific | cbox-plastic |
|-------------|--------------|--------------|
| ![cornell-box](gallery/cornell-box.png) | ![cbox-pacific](gallery/cbox-pacific.png) | ![cbox-plastic](gallery/cbox-plastic.png) |

### MatPreview

| matpreview-conductor | matpreview-dielectric | matpreview-diffuse | matpreview-plastic |
|----------------------|----------------------|--------------------|--------------------|
| ![matpreview-conductor](gallery/matpreview-conductor.png) | ![matpreview-dielectric](gallery/matpreview-dielectric.png) | ![matpreview-diffuse](gallery/matpreview-diffuse.png) | ![matpreview-plastic](gallery/matpreview-plastic.png) |

| matpreview-roughconductor | matpreview-roughdielectric | thindielectric |
|--------------------------|----------------------------|----------------|
| ![matpreview-roughconductor](gallery/matpreview-roughconductor.png) | ![matpreview-roughdielectric](gallery/matpreview-roughdielectric.png) | ![thindielectric](gallery/thindielectric.png) |

### Integrators comparison

| path-trace(spp=32) | pathtrace-unstr(spp=32) | particle-trace(spp=32) | bidir-path-trace(spp=32) |
|-----|---|-----|-----|
| ![pathtrace32](gallery/cornell-box-pathtrace-spp32.png) | ![pathtrace_unstrat32](gallery/cornell-box-pathtrace_unstrat-spp32.png) | ![ptrace32](gallery/cornell-box-ptrace-spp32.png) | ![bidir32](gallery/cornell-box-bidir-spp32.png) |

| path-trace(spp=2048) | particle-trace(spp=2048) |
|----------------------|--------------------------|
| ![pathtrace2048](gallery/veach-ajar-pathtrace-spp2048.png) | ![ptrace2048](gallery/veach-ajar-ptrace-spp2048.png) |

### Misc.

| car | car2 |
|-----|------|
| ![car](gallery/car.png) | ![car2](gallery/car2.png) |

| veach-ajar | glass-of-water |
|------------|----------------|
| ![veach-ajar](gallery/veach-ajar.png) | ![glass-of-water](gallery/glass-of-water.png) |

| lamp | lego-bulldozer | stanford-bunny-dielectric |
|------|---------------|---------------------------|
| ![lamp](gallery/lamp.png) | ![lego-bulldozer](gallery/lego-bulldozer.png) | ![stanford-bunny-dielectric](gallery/stanford-bunny-dielectric.png) |

| veach-mis | teapot |
|---------------|--------|
| ![veach-mis](gallery/veach-mis.png) | ![teapot](gallery/teapot.png) |

| stanford-bunny-diffuse |                stanford-dragon                  |
| ---------------------- |-------------------------------------------------|
| ![stanford-bunny-diffuse](gallery/stanford-bunny-diffuse.png) | ![stanford-dragon](gallery/stanford-dragon.png) |

---

## License

PacificRenderer is open source and distributed under the MIT License.

---

## Acknowledgements

- Mitsuba Renderer
- PBRT
- OpenEXR, stb, pugixml, tinyobjloader, and other open-source libraries
- See `ext/` for third-party library licenses

---

For more information, see the code and comments, or open an issue!
