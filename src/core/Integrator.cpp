#include "core/Integrator.h"

#include <algorithm>
#include <chrono>
#include <iostream>

#include "core/Thread.h"

void SamplingIntegrator::render(const Scene *scene, Sensor *sensor, uint32_t n_threads, bool show_progress) {
    uint32_t width = sensor->film.width;
    uint32_t height = sensor->film.height;
    uint32_t total_pixels = width * height;

    // Use sensor->sampler as the master RNG, then create n_threads Samplers with different seeds
    ThreadPool tpool{sensor->sampler, n_threads};
    std::vector<std::future<void>> results;
    std::atomic<size_t> n_rendered_pixels{0};
    std::mutex print_mutex;

    auto start_time = std::chrono::high_resolution_clock::now();

    for (int row = 0; row < height; row++) {
        break;
        for (int col = 0; col < width; col++) {
            // TODO: debug
            // if (row <= -1 || row >= 60 || col <= -1 || col >= 60)
            //     continue;
            for (size_t i = 0; i < sensor->sampler.spp; i++) {
                Float px, py;
                Ray sensor_ray = sensor->sample_ray(row, col, sensor->sampler.get_2D(), px, py);
                Vec3f returned_radiance = this->sample_radiance(scene, &sensor->sampler, sensor_ray);
                sensor->film.commit_sample(returned_radiance, row, col, px, py);
            }
            if (show_progress)
                if((row * height + col + 1) % 100 == 0 || (row == height - 1 && col == width - 1))
                    std::cout << "\rProgress: " << std::format("{:.02f}", ((row * width + col + 1) / static_cast<double>(total_pixels)) * 100) << "%" << std::flush;
        }
    }

    uint32_t block_size = 16;
    uint32_t n_row_blocks = (height - 1) / block_size + 1;
    uint32_t n_col_blocks = (width - 1) / block_size + 1;
    for (uint32_t block_row = 0; block_row < n_row_blocks; block_row++) {
        for (uint32_t block_col = 0; block_col < n_col_blocks; block_col++) {
            results.emplace_back(
                tpool.enqueue([this, block_row, block_col, scene, sensor, &print_mutex, &n_rendered_pixels, total_pixels, block_size, width, height, show_progress](Sampler &sampler) {
                    uint32_t row_bound = std::min(block_size, height - block_row * block_size);
                    uint32_t col_bound = std::min(block_size, width - block_col * block_size);
                    for (size_t inblock_row = 0; inblock_row < row_bound; inblock_row++) {
                        for (size_t inblock_col = 0; inblock_col < col_bound; inblock_col++) {
                            uint32_t row = inblock_row + block_size * block_row;
                            uint32_t col = inblock_col + block_size * block_col;

                            // TODO: for debug purposes
                            // if (row != 0 || col != 0)
                            //     continue;
                                
                            for (size_t i = 0; i < sensor->sampler.spp; i++) {
                                // sample position in sensor space ([0, 1])
                                // sample position in sensor space ([0, 1])
                                Float px, py;
                                Ray sensor_ray = sensor->sample_ray(row, col, sampler.get_2D(), px, py);
                                Vec3f returned_radiance = this->sample_radiance(scene, &sampler, sensor_ray);

                                // check for invalid values
                                if (std::isnan(returned_radiance.x) || std::isnan(returned_radiance.y) || std::isnan(returned_radiance.z) ||
                                    std::isinf(returned_radiance.x) || std::isinf(returned_radiance.y) || std::isinf(returned_radiance.z) ||
                                    returned_radiance.x < 0 || returned_radiance.y < 0 || returned_radiance.z < 0)
                                    throw std::runtime_error("Invalid radiance value: " + std::to_string(returned_radiance.x) + ", " + std::to_string(returned_radiance.y) + ", " + std::to_string(returned_radiance.z) + " at pixel (" + std::to_string(row) + ", " + std::to_string(col) + ")");

                                sensor->film.commit_sample(returned_radiance, row, col, px, py);
                            }
                        }
                    }

                    // Update progress
                    if (show_progress) {
                        n_rendered_pixels.fetch_add(row_bound * col_bound);
                        size_t completed = n_rendered_pixels;
                        {
                            std::lock_guard<std::mutex> lock(print_mutex);
                            std::cout << "\rProgress: " << std::format("{:.02f}", (completed / static_cast<double>(total_pixels)) * 100) << "%" << std::flush;
                        }
                    }
                }));
        }
    }
    for (auto &result : results)
        result.get();
    std::cout << std::endl;

    sensor->film.normalize_pixels();

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "Rendering completed in " << elapsed.count() << " seconds.";
}

Float SamplingIntegrator::get_mis_weight_nee(const Intersection &isc, const EmitterSample &emitter_sample) const {
    if (emitter_sample.emitter_flags == EmitterFlags::DELTA_DIRECTION)
        return 1.0;
    if (!isc.shape->bsdf->has_flag(BSDFFlags::Delta)) {
        Vec3f wi_local = worldToLocal(isc.dirn, isc.normal);
        Vec3f wo_local = worldToLocal(-emitter_sample.direction, isc.normal);
        Float bsdf_sampling_pdf = isc.shape->bsdf->pdf(wi_local, wo_local);
        return Sqr(emitter_sample.pdf) / (Sqr(emitter_sample.pdf) + Sqr(bsdf_sampling_pdf));
    }

    return 0.0;
}

Float SamplingIntegrator::get_mis_weight_bsdf(const Scene *scene, const Intersection &isc, const BSDFSample &bsdf_sample) const {
    if (isc.shape->bsdf->has_flag(BSDFFlags::Delta))
        return 1.0;
    Float nee_pdf = scene->pdf_nee(isc, localToWorld(bsdf_sample.wo, isc.normal));
    return Sqr(bsdf_sample.pdf) / (Sqr(bsdf_sample.pdf) + Sqr(nee_pdf));
}
