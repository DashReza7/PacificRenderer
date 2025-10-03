#include "core/Integrator.h"

#include <chrono>
#include <iostream>
#include <algorithm>

#include "core/Thread.h"


void SamplingIntegrator::render(const Scene *scene, Sensor *sensor, uint32_t n_threads) {
    uint32_t width = sensor->film.width;
    uint32_t height = sensor->film.height;
    uint32_t total_pixels = width * height;

    // Use sensor->sampler as the master RNG, then create n_threads Samplers with different seeds
    ThreadPool tpool{sensor->sampler, n_threads};
    std::vector<std::future<void>> results;
    std::atomic<size_t> n_rendered_pixels{0};
    std::mutex print_mutex;

    auto start_time = std::chrono::high_resolution_clock::now();

    // for (size_t row = 0; row < height; row++) {
    //     for (size_t col = 0; col < width; col++) {
    //         Vec3f radiance{0.0};
    //         for (size_t i = 0; i < sensor->sampler.spp; i++) {
    //             Ray sensor_ray = sensor->sample_ray(row, col, sensor->sampler.get_sample_2d());
    //             Vec3f returned_radiance = this->sample_radiance(scene, &sensor->sampler, sensor_ray);
    //             radiance += returned_radiance;
    //         }
    //         radiance /= sensor->sampler.spp;
    //         sensor->film.set_pixel(row, col, radiance);
    //         if ((row * height + col + 1) % 100 == 0 || (row == height - 1 && col == width - 1))
    //             std::cout << "\rCompleted " << row * height + col + 1 << " / " << total_pixels << " pixels." << std::flush;
    //     }
    // }

    uint32_t block_size = 16;
    uint32_t n_row_blocks = (height - 1) / block_size + 1;
    uint32_t n_col_blocks = (width - 1) / block_size + 1;
    for (uint32_t block_row = 0; block_row < n_row_blocks; block_row++) {
        for (uint32_t block_col = 0; block_col < n_col_blocks; block_col++) {
            results.emplace_back(
                tpool.enqueue([this, block_row, block_col, scene, sensor, &print_mutex, &n_rendered_pixels, total_pixels, block_size, width, height](Sampler &sampler) {
                    uint32_t row_bound = std::min(block_size, height - block_row * block_size);
                    uint32_t col_bound = std::min(block_size, width - block_col * block_size);
                    for (size_t inblock_row = 0; inblock_row < row_bound; inblock_row++) {
                        for (size_t inblock_col = 0; inblock_col < col_bound; inblock_col++) {
                            uint32_t row = inblock_row + block_size * block_row;
                            uint32_t col = inblock_col + block_size * block_col;
                            Vec3f radiance{0.0};
                            for (size_t i = 0; i < sensor->sampler.spp; i++) {
                                Ray sensor_ray = sensor->sample_ray(row, col, sampler.get_sample_2d());
                                Vec3f returned_radiance = this->sample_radiance(scene, &sampler, sensor_ray);
                                radiance += returned_radiance;
                            }
                            radiance /= sensor->sampler.spp;
                            sensor->film.set_pixel(row, col, radiance);
                        }
                    }

                    // Update progress
                    n_rendered_pixels.fetch_add(row_bound * col_bound);
                    size_t completed = n_rendered_pixels;
                    {
                        std::lock_guard<std::mutex> lock(print_mutex);
                        std::cout << "\rProgress: " << std::format("{:.02f}", (completed / static_cast<double>(total_pixels)) * 100) << "%" << std::flush;
                    }
                }));
        }
    }
    for (auto &result : results)
        result.get();
    std::cout << std::endl;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "Rendering completed in " << elapsed.count() << " seconds.";
}
