#include "core/Integrator.h"
#include "core/Thread.h"

class PrimarySample {
private:
    int cur_coord = 0;  // used for accessing the next sample requested by integrator
    int mut_cnt = 0;    // number of accepted mutations since the latest large step event
    std::vector<Float> tent_samples{};
    std::vector<int> samples_mut_cnt{};  // number of accepted mutations of each sample since the last large step event
    static constexpr Float mut_std = 0.01;

    // small mutation given a current sample
    Float mutate(Float cur_sample) {
        Float u1 = sampler->get_1D(), u2 = sampler->get_1D();
        while (u1 == 0)
            u1 = sampler->get_1D();
        Float z0 = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * Pi * u2);
        cur_sample = cur_sample + z0 * mut_std;
        while (cur_sample >= 1.0)
            cur_sample -= 1.0;
        while (cur_sample < 0)
            cur_sample += 1.0;
        return cur_sample;
    }

public:
    Sampler *sampler;
    std::vector<Float> samples{};

    PrimarySample(Sampler *sampler) : sampler(sampler) {
        // TODO
        // iniit samples (all uniform)
        samples.resize(100);
        for (int i = 0; i < samples.size(); i++)
            samples[i] = sampler->get_1D();
        tent_samples = std::vector<Float>(samples);
    }

    void makeNewTentSamples(bool large_step) {
        cur_coord = 0;
        if (large_step)
            for (int i = 0; i < tent_samples.size(); i++)
                tent_samples[i] = sampler->get_1D();
        else
            for (int i = 0; i < tent_samples.size(); i++)
                tent_samples[i] = mutate(tent_samples.at(i));
    }

    // get the next sample. Handle the lazy perterbations
    Float getSample(bool large_step) {
        // TODO:
        Float rnd = tent_samples.at(cur_coord);
        cur_coord++;
        return rnd;


        tent_samples.push_back(0);
        if (large_step) {
            tent_samples[cur_coord] = sampler->get_1D();
        } else {  // small step
            Float old_sample = cur_coord >= samples.size() ? sampler->get_1D() : samples.at(cur_coord);
            int compensate_mut_cnt = mut_cnt - (cur_coord >= samples.size() ? 0 : samples_mut_cnt.at(cur_coord));
            for (int i = 0; i < compensate_mut_cnt; i++)
                old_sample = mutate(old_sample);
            // apply the small perturbation
            tent_samples[cur_coord] = mutate(old_sample);
        }

        cur_coord++;
        return tent_samples.at(cur_coord - 1);
    }

    void commit(bool accept, bool large_step) {
        cur_coord = 0;
        if (accept)
            samples = std::vector<Float>(tent_samples);
        else
            tent_samples = samples;
        return;

        if (accept) {
            if (large_step) {
                mut_cnt = 0;
                for (auto &i : samples_mut_cnt)
                    i = 0;
                for (int i = samples_mut_cnt.size(); i < tent_samples.size(); i++)
                    samples_mut_cnt.push_back(0);
                for (int i = tent_samples.size(); i < samples.size(); i++)  // the rest is handled in the last for loop
                    samples[i] = sampler->get_1D();
            } else {  // small step
                mut_cnt++;
                for (int i = 0; i < tent_samples.size(); i++)
                    if (i < samples_mut_cnt.size())
                        samples_mut_cnt[i] = mut_cnt;
                    else
                        samples_mut_cnt.push_back(mut_cnt);
            }
            for (int i = 0; i < tent_samples.size(); i++)
                if (i < samples.size())
                    samples[i] = tent_samples.at(i);
                else
                    samples.push_back(tent_samples.at(i));
        } else {  // mut not accepted
        }
        tent_samples.clear();
        cur_coord = 0;
    }
};

class PSSMLTIntegrator : public Integrator {
private:
    int b_samples;  // number of path samples to estimate the normalization factor
    int n_seeds;
    int chain_steps;
    int max_depth;
    int rr_depth;
    bool hide_emitters;
    Float p_large;

    Float estimate_b(const Scene *scene, Sampler *sampler, size_t n_threads, Float &estimate_time, bool show_progress) const;
    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const;
    std::vector<std::pair<PrimarySample, Vec3f>> init_seeds(const Scene *scene, Sensor *sensor) const;
    Vec3f sample_path(const Scene *scene, PrimarySample &ps, const Ray &ray, bool large_step = true) const;
    void markovChainLoop(int tseed_start_idx, int seeds_per_thread,
                         std::vector<std::pair<PrimarySample, Vec3f>> &seeds, Sampler &sampler, 
                         Sensor *sensor, const Scene *scene, Float inv_b, 
                         std::atomic<int> &nseeds_completed, std::mutex &print_mutex) const;

    Float get_mis_weight_nee(const Intersection &isc, const EmitterSample &emitter_sample, uint32_t n_bsdf_samples) const {
        if ((emitter_sample.emitter_flags & EmitterFlags::DELTA_DIRECTION) != EmitterFlags::NONE || n_bsdf_samples == 0)
            return 1.0;
        if (!isc.shape->bsdf->has_flag(BSDFFlags::Delta)) {
            Vec3f wo_local = worldToLocal(-emitter_sample.direction, isc.normal);
            Float bsdf_sampling_pdf = isc.shape->bsdf->pdf(isc, wo_local);
            if (bsdf_sampling_pdf < 0.0)
                throw std::runtime_error("Negative BSDF pdf in MIS weight computation");

            return Sqr(emitter_sample.pdf) / (Sqr(emitter_sample.pdf) + Sqr(bsdf_sampling_pdf));
        }

        return 0.0;
    }
    Float get_mis_weight_bsdf(const Scene *scene, const Intersection &isc, const BSDFSample &bsdf_sample, uint32_t n_emitter_samples) const {
        if ((bsdf_sample.flags & BSDFSampleFlags::Delta) != BSDFSampleFlags::None || n_emitter_samples == 0)
            return 1.0;
        Float nee_pdf = scene->pdf_nee(isc, localToWorld(bsdf_sample.wo, isc.normal));
        if (nee_pdf < 0.0)
            throw std::runtime_error("Negative NEE pdf in MIS weight computation");
        if (nee_pdf <= Epsilon)
            return 1.0;

        return Sqr(bsdf_sample.pdf) / (Sqr(bsdf_sample.pdf) + Sqr(nee_pdf));
    }

public:
    PSSMLTIntegrator(int b_samples, int n_seeds, int chain_steps, int max_depth, int rr_depth, bool hide_emitters, Float p_large) : b_samples(b_samples), n_seeds(n_seeds), chain_steps(chain_steps), max_depth(max_depth), rr_depth(rr_depth), hide_emitters(hide_emitters), p_large(p_large) {}

    void render(const Scene *scene, Sensor *sensor, uint32_t n_threads, bool show_progress) override;
    std::string to_string() const override {
        throw std::runtime_error("PSSMLT to_string not implemented yet!");
    }
};

// ------------------ PSSMLT function definitions ----------------------------
void PSSMLTIntegrator::render(const Scene *scene, Sensor *sensor, uint32_t n_threads, bool show_progress) {
    // Estimate b
    Float b_estimate_time;
    Float b = estimate_b(scene, &sensor->sampler, n_threads, b_estimate_time, show_progress);
    std::cout << "b estimation took " << std::format("{:.02f}", b_estimate_time) << " seconds." << std::endl;
    Float inv_b = 1.0 / b;

    // Initialize path seeds
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<std::pair<PrimarySample, Vec3f>> seeds = init_seeds(scene, sensor);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "seed initialization took " << std::format("{:.02f}", elapsed.count()) << " seconds.\n";

    // Markov Chain loop
    start_time = std::chrono::high_resolution_clock::now();
    ThreadPool tpool{sensor->sampler, n_threads};
    std::vector<std::future<void>> results;
    std::atomic<int> nseeds_completed = 0;
    std::mutex print_mutex;
    for (int tidx = 0; tidx < n_threads; tidx++) {
        int seeds_per_thread = seeds.size() / n_threads;
        int tseed_start_idx = tidx * seeds_per_thread;
        if (tidx == n_threads - 1)
            seeds_per_thread += seeds.size() % n_threads;
        results.emplace_back(tpool.enqueue([this, tidx, sensor, scene, inv_b, tseed_start_idx, seeds_per_thread, &seeds, &nseeds_completed, &print_mutex](Sampler &sampler) {
            markovChainLoop(tseed_start_idx, seeds_per_thread, seeds, sampler, sensor, scene, inv_b, nseeds_completed, print_mutex);
        }));
    }
    for (auto &result : results)
        result.get();
    end_time = std::chrono::high_resolution_clock::now();
    elapsed = end_time - start_time;
    std::cout << "\nMarkov Chain time: " << std::format("{:.02f}", elapsed.count()) << " seconds.\n";

    sensor->film.normalize_pixels(sensor->film.width * sensor->film.height);
}

void PSSMLTIntegrator::markovChainLoop(int tseed_start_idx, int seeds_per_thread,
                                       std::vector<std::pair<PrimarySample, Vec3f>> &seeds, Sampler &sampler, 
                                       Sensor *sensor, const Scene *scene, Float inv_b, 
                                       std::atomic<int> &nseeds_completed, std::mutex &print_mutex) const {

    for (int i = tseed_start_idx; i < tseed_start_idx + seeds_per_thread; i++) {
        seeds[i].first.sampler = &sampler;
        for (int step = 0; step < chain_steps; step++) {
            bool large_step = sampler.get_1D() < p_large;
            seeds[i].first.makeNewTentSamples(large_step);

            Vec2f pfilm{seeds[i].first.getSample(large_step), seeds[i].first.getSample(large_step)};
            Ray sensor_ray{sensor->origin_world, sensor->iplaneToWorld(pfilm.x, pfilm.y), 1e-3, 1e6};
            Vec3f tent_radiance = sample_path(scene, seeds[i].first, sensor_ray, large_step);
            Float I_new = average(tent_radiance);
            Float I_old = average(seeds[i].second);
            // handle the case where one or both of them are zero
            Float a = (I_old == 0 && I_new != 0) ?   1 : 
                  (I_old == 0 && I_new == 0)     ? 0.5 : std::max(std::min(1.0f, I_new / I_old), 0.0f);
            Vec3f old_contrib = (1.0f - a) * seeds[i].second / ((I_old * inv_b + p_large) * chain_steps * seeds.size());
            Vec3f new_contrib = (a + (large_step ? 1 : 0)) * tent_radiance / ((I_new * inv_b + p_large) * chain_steps * seeds.size());
            // Vec3f old_contrib = Float(1.0 - a) * seeds[i].second / I_old;
            // Vec3f new_contrib = Float(a)       * tent_radiance   / I_new;
            sensor->film.commit_splat(old_contrib, Vec2f{seeds[i].first.samples.at(0), seeds[i].first.samples.at(1)});
            sensor->film.commit_splat(new_contrib, pfilm);

            bool accept = sampler.get_1D() < a;
            seeds[i].first.commit(accept, large_step);
            if (accept)
                seeds[i].second = tent_radiance;
        }
        nseeds_completed.fetch_add(1);
        {
            std::lock_guard<std::mutex> lock(print_mutex);
            std::cout << "\rMarkov Chain progress: " << std::format("{:3.02f}%", Float(nseeds_completed) / seeds.size() * 100);
        }
    }
}

Float PSSMLTIntegrator::estimate_b(const Scene *scene, Sampler *sampler, size_t n_threads, Float &estimate_time, bool show_progress) const {
    std::atomic<Float> b{0};

    ThreadPool tpool{*sampler, n_threads};
    std::vector<std::future<void>> results;

    auto start_time = std::chrono::high_resolution_clock::now();
    for (int tidx = 0; tidx < n_threads; tidx++) {
        int samples_per_thread = b_samples / n_threads;
        if (tidx == n_threads - 1)
            samples_per_thread += b_samples % n_threads;
        results.emplace_back(tpool.enqueue([this, samples_per_thread, scene, &b](Sampler &sampler) {
            Float b_thread = 0;
            for (int smpl = 0; smpl < samples_per_thread; smpl++) {
                Vec2f p_film = sampler.get_2D();
                Vec3f sensor_dirn = scene->sensor->iplaneToWorld(p_film.x, p_film.y);
                Ray sensor_ray{scene->sensor->origin_world, sensor_dirn, 1e-3, 1e6};
                Vec3f incoming_radiance = sample_radiance(scene, &sampler, sensor_ray, -1, -1);
                b_thread += average(incoming_radiance) / b_samples;
            }
            b.fetch_add(b_thread);
        }));
    }
    for (auto &result : results)
        result.get();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    estimate_time = elapsed.count();

    return b;
}

Vec3f PSSMLTIntegrator::sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const {
    if (max_depth == 0)
        return Vec3f{0.0};
    if (max_depth < -1)
        throw std::runtime_error("max_depth must be -1 (infinite) or a non-negative integer");

    Vec3f throughput{1.0};
    Vec3f radiance{0.0};

    Ray curr_ray = ray;
    Intersection curr_isc;
    bool is_hit = scene->ray_intersect(curr_ray, curr_isc);

    // ----------------------- Visible emitters -----------------------
    if (!hide_emitters) {
        // envmap
        if (!is_hit) {
            if (scene->env_map == nullptr)
                return Vec3f{0.0};
            curr_isc.dirn = curr_ray.d;
            // TODO: BUG for hdr envmap maybe?
            Vec3f lightLi = scene->env_map->eval(curr_isc);
            if (std::isnan(lightLi.x) || std::isnan(lightLi.y) || std::isnan(lightLi.z))
                throw std::runtime_error("Env map returned NaN value in PathTracerIntegrator");
            if (std::isinf(lightLi.x) || std::isinf(lightLi.y) || std::isinf(lightLi.z))
                throw std::runtime_error("Env map returned Inf value in PathTracerIntegrator");
            return lightLi;
        }
        // area light
        if (curr_isc.shape->emitter && glm::dot(curr_isc.normal, curr_isc.dirn) > 0.0)
            radiance += throughput * curr_isc.shape->emitter->eval(curr_isc);
    }
    // blackout emitters if hide_emitter
    if (is_hit && hide_emitters && curr_isc.shape->emitter)
        return Vec3f{0};

    for (int depth = 1; depth < max_depth || max_depth == -1; depth++) {
        if (!is_hit ||
            (glm::dot(curr_isc.dirn, curr_isc.normal) <= 0.0 && !curr_isc.shape->bsdf->has_flag(BSDFFlags::TwoSided) && !curr_isc.shape->bsdf->has_flag(BSDFFlags::PassThrough)))
            break;

        // ----------------------- Emitter sampling -----------------------

        if (!curr_isc.shape->bsdf->has_flag(BSDFFlags::Delta)) {
            EmitterSample emitter_sample = scene->sample_emitter(curr_isc, sampler->get_1D(), sampler->get_3D());
            if (emitter_sample.is_visible) {
                Vec3f wo_local = worldToLocal(-emitter_sample.direction, curr_isc.normal);
                Vec3f bsdf_value = curr_isc.shape->bsdf->eval(curr_isc, wo_local);

                // TODO: debug
                if (bsdf_value.x < 0.0 || bsdf_value.y < 0.0 || bsdf_value.z < 0.0)
                    throw std::runtime_error("BSDF eval returned non-positive value in DirectLightingIntegrator");
                if (std::isnan(bsdf_value.x) || std::isnan(bsdf_value.y) || std::isnan(bsdf_value.z))
                    throw std::runtime_error("BSDF eval returned NaN value in DirectLightingIntegrator");
                if (std::isinf(bsdf_value.x) || std::isinf(bsdf_value.y) || std::isinf(bsdf_value.z))
                    throw std::runtime_error("BSDF eval returned Inf value in DirectLightingIntegrator");

                Float mis_weight = get_mis_weight_nee(curr_isc, emitter_sample, 1);
                radiance += mis_weight * throughput * emitter_sample.radiance * bsdf_value / emitter_sample.pdf;
            }
        }

        // ------------------------ BSDF sampling -------------------------

        auto [bsdf_sample, bsdf_value] = curr_isc.shape->bsdf->sample(curr_isc, sampler->get_1D(), sampler->get_2D());
        if (bsdf_sample.pdf < 0.0 || bsdf_value.x < 0.0 || bsdf_value.y < 0.0 || bsdf_value.z < 0.0) {
            std::cerr << curr_isc.shape->bsdf->to_string() << std::endl;
            throw std::runtime_error("BSDF sample returned invalid value in PathTracerIntegrator: " + std::to_string(bsdf_sample.pdf) + ", " + std::to_string(bsdf_value.x) + ", " + std::to_string(bsdf_value.y) + ", " + std::to_string(bsdf_value.z));
        }
        if (std::isnan(bsdf_sample.pdf) || std::isnan(bsdf_value.x) || std::isnan(bsdf_value.y) || std::isnan(bsdf_value.z))
            throw std::runtime_error("BSDF sample returned NaN value in PathTracerIntegrator");
        if (std::isinf(bsdf_sample.pdf) || std::isinf(bsdf_value.x) || std::isinf(bsdf_value.y) || std::isinf(bsdf_value.z))
            throw std::runtime_error("BSDF sample returned Inf value in PathTracerIntegrator");
        if (bsdf_sample.pdf <= Epsilon || glm::length(bsdf_value) <= Epsilon)
            break;

        Float mis_weight = get_mis_weight_bsdf(scene, curr_isc, bsdf_sample, 1);
        throughput *= mis_weight * bsdf_value / bsdf_sample.pdf;

        if (std::isnan(throughput.x) || std::isnan(throughput.y) || std::isnan(throughput.z))
            throw std::runtime_error("Throughput is NaN in PathTracerIntegrator");
        if (std::isinf(throughput.x) || std::isinf(throughput.y) || std::isinf(throughput.z))
            throw std::runtime_error("Throughput is Inf in PathTracerIntegrator");

        curr_ray = Ray{curr_isc.position + sign(glm::dot(localToWorld(bsdf_sample.wo, curr_isc.normal), curr_isc.normal)) * curr_isc.normal * Epsilon, localToWorld(bsdf_sample.wo, curr_isc.normal), Epsilon, 1e4};
        is_hit = scene->ray_intersect(curr_ray, curr_isc);
        Vec3f lightLi{0.0};
        if (is_hit) {
            if (curr_isc.shape->emitter == nullptr || glm::dot(curr_isc.dirn, curr_isc.normal) < 0)
                lightLi = Vec3f{0.0};
            else
                lightLi = curr_isc.shape->emitter->eval(curr_isc);
        } else {
            if (scene->env_map != nullptr) {
                Intersection tmp;
                tmp.dirn = curr_ray.d;
                lightLi = scene->env_map->eval(tmp);
            }
        }
        radiance += throughput * lightLi;

        // do RussianRoulette
        if (depth + 1 >= rr_depth) {
            Float rr_survive_prob = std::min(std::max(throughput.x, std::max(throughput.y, throughput.z)), Float(0.95));
            if (sampler->get_1D() > rr_survive_prob)
                break;
            throughput /= rr_survive_prob;
        }
    }

    return radiance;
}

// return a list of primarySamples with their corresponding contribs
std::vector<std::pair<PrimarySample, Vec3f>> PSSMLTIntegrator::init_seeds(const Scene *scene, Sensor *sensor) const {
    // list of (ps, scalar_contrib). used for sampling them according to their scalar_contrib
    int n_candids = int(n_seeds * 100);
    std::vector<std::pair<PrimarySample, Vec3f>> candids{};
    candids.reserve(n_candids);
    for (int i = 0; i < n_candids; i++) {
        // sample a (several) lightpath based on a sequence of random numbers and return their max contribution
        PrimarySample ps{&sensor->sampler};
        Vec2f pfilm{ps.getSample(true), ps.getSample(true)};
        Ray sensor_ray{sensor->origin_world, sensor->iplaneToWorld(pfilm.x, pfilm.y), 1e-3, 1e6};
        Vec3f radiance = sample_path(scene, ps, sensor_ray);
        ps.commit(true, true);
        candids.push_back({ps, radiance});
    }

    // sample n_seeds seeds from candids based on their scalar_contribution (candids[i].second.average)
    std::vector<std::pair<PrimarySample, Vec3f>> selected_candids;
    std::vector<Float> cdf(n_candids + 1, 0.0f);
    for (int i = 0; i < n_candids; i++)
        cdf[i + 1] = cdf[i] + average(candids[i].second);
    Float total_weight = cdf[n_candids];
    if (total_weight == 0.0f) {
        for (int i = 0; i < n_seeds; i++)
            selected_candids.push_back(candids[i % n_candids]);
        return selected_candids;
    }
    for (int i = 1; i <= n_candids; i++)
        cdf[i] /= total_weight;
    selected_candids.reserve(n_seeds);
    for (int i = 0; i < n_seeds; i++) {
        Float u = sensor->sampler.get_1D();
        // Binary search to find the index in the CDF
        int idx = int(std::lower_bound(cdf.begin(), cdf.end(), u) - cdf.begin()) - 1;
        idx = std::clamp(idx, 0, n_candids - 1);
        selected_candids.push_back(candids[idx]);
    }

    return selected_candids;
}

Vec3f PSSMLTIntegrator::sample_path(const Scene *scene, PrimarySample &ps, const Ray &ray, bool large_step) const {
    Vec3f throughput{1.0};
    Vec3f radiance{0.0};

    Ray curr_ray = ray;
    Intersection curr_isc;
    bool is_hit = scene->ray_intersect(curr_ray, curr_isc);

    // ----------------------- Visible emitters -----------------------
    if (is_hit && curr_isc.shape->emitter && hide_emitters)
        return Vec3f{0};

    for (int depth = 1; depth < max_depth || max_depth == -1; depth++) {
        if (!is_hit) {  // FIXME: currently doesn't check for hide_emitters
            if (scene->env_map != nullptr) {
                curr_isc.dirn = curr_ray.d;
                radiance += throughput * scene->env_map->eval(curr_isc);
            }
            break;
        }
        if (!BSDF::frontSide(curr_isc))
            break;

        // ----------------------- Emitter sampling -----------------------
        if (!curr_isc.shape->bsdf->has_flag(BSDFFlags::Delta)) {
            EmitterSample emitter_sample = scene->sample_emitter(curr_isc, ps.getSample(large_step), Vec3f{ps.getSample(large_step), ps.getSample(large_step), ps.getSample(large_step)});
            if (emitter_sample.is_visible) {
                Vec3f wo_local = worldToLocal(-emitter_sample.direction, curr_isc.normal);
                Vec3f bsdf_value = curr_isc.shape->bsdf->eval(curr_isc, wo_local);
                Float mis_weight = get_mis_weight_nee(curr_isc, emitter_sample, 1);
                // Float mis_weight = 1;
                radiance += mis_weight * throughput * emitter_sample.radiance * bsdf_value / emitter_sample.pdf;
                if (!check_valid(radiance))
                    throw std::runtime_error("radiance invalid at Emitter Sampling");
                // if (ps.getSample(large_step) >= 0.3)
                //     return radiance / Float(0.7);
                // else {
                //     radiance = Vec3f{0};
                //     throughput /= Float(0.3);
                // }
            }
        }

        // ------------------------ BSDF sampling -------------------------
        auto [bsdf_sample, bsdf_value] = curr_isc.shape->bsdf->sample(curr_isc, ps.getSample(large_step), Vec2f{ps.getSample(large_step), ps.getSample(large_step)});
        if (bsdf_sample.pdf <= Epsilon)
            break;
        Float mis_weight = get_mis_weight_bsdf(scene, curr_isc, bsdf_sample, 1);
        // Float mis_weight = 1;
        throughput *= mis_weight * bsdf_value / bsdf_sample.pdf;
        if (!check_valid(throughput))
            throw std::runtime_error("throughput invalid at BSDF Sampling");

        curr_ray = Ray{curr_isc.position + sign(glm::dot(localToWorld(bsdf_sample.wo, curr_isc.normal), curr_isc.normal)) * curr_isc.normal * Epsilon, localToWorld(bsdf_sample.wo, curr_isc.normal), Epsilon, 1e4};
        is_hit = scene->ray_intersect(curr_ray, curr_isc);

        // FIXME: currently doesn't check for the Back of the emitter
        if (is_hit && curr_isc.shape->emitter && glm::dot(curr_isc.normal, curr_ray.d) < 0)
            radiance += curr_isc.shape->emitter->eval(curr_isc) * throughput;
        if (!check_valid(radiance))
            throw std::runtime_error("radiance invalid at BSDF Sampling");
    }
    return radiance;
}

// ------------------- Registry functions -------------------
Integrator *createPSSMLTIntegrator(const std::unordered_map<std::string, std::string> &properties) {
    int b_samples = 10000;
    int n_seeds = 100;
    int chain_steps = 1024;
    int max_depth = 20;
    int rr_depth = 5;
    bool hide_emitters = false;
    Float p_large = 0.3;

    for (const auto &[key, value] : properties) {
        if (key == "b_samples") {
            b_samples = std::stoi(value);
        } else if (key == "n_seeds") {
            n_seeds = std::stoi(value);
        } else if (key == "chain_steps") {
            chain_steps = std::stoi(value);
        } else if (key == "max_depth") {
            max_depth = std::stoi(value);
        } else if (key == "rr_depth") {
            rr_depth = std::stoi(value);
        } else if (key == "hide_emitters") {
            hide_emitters = (value == "true" || value == "1");
        } else if (key == "p_large") {
            p_large = std::stod(value);
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Path Tracer integrator");
        }
    }

    return new PSSMLTIntegrator(b_samples, n_seeds, chain_steps, max_depth, rr_depth, hide_emitters, p_large);
}
namespace {
struct PSSMLTIntegratorRegistrar {
    PSSMLTIntegratorRegistrar() {
        IntegratorRegistry::registerIntegrator("pssmlt", createPSSMLTIntegrator);
    }
};

static PSSMLTIntegratorRegistrar registrar;
}  // namespace
