#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <queue>

constexpr double G = 9.8;

class Stage {
 public:
    Stage() = default;
    Stage(int64_t S, int64_t L, int64_t T, int64_t C)
    : stage_mass_(S)
    , fuel_mass_(L)
    , thrust_(T)
    , consumption_(C) {}

    double ComputeVelocityAdd(int64_t payload_mass) const {
        double burn_time = static_cast<double>(fuel_mass_) / consumption_;
        int64_t full_perm_mass = stage_mass_ + payload_mass;
        return (static_cast<double>(thrust_) / consumption_)
            * (std::log(TotalMass(payload_mass)) - std::log(full_perm_mass)) - G * burn_time;
    }

    bool IsUseless(int64_t payload_mass) const {
        return static_cast<double>(thrust_) / TotalMass(payload_mass) <= G;
    }

    int64_t TotalMass(int64_t payload_mass) const {
        return stage_mass_ + fuel_mass_ + payload_mass;
    }

 private:
    int64_t stage_mass_;
    int64_t fuel_mass_;
    int64_t thrust_;
    int64_t consumption_;
};

std::istream& operator>>(std::istream& is, Stage& s) {
    int64_t S, L, T, C;
    is >> S >> L >> T >> C;
    assert(S > 0 && S <= std::numeric_limits<uint32_t>::max());
    assert(L > 0 && L <= std::numeric_limits<uint32_t>::max());
    assert(T > 0 && T <= std::numeric_limits<uint32_t>::max());
    assert(C > 0 && C <= std::numeric_limits<uint32_t>::max());
    s = Stage(S, L, T, C);
    return is;
}

int main(int argc, char** argv) {
    int N;
    std::cin >> N;
    assert(N > 0 && N <= 1000);

    std::unordered_map<int64_t, double> mass2velocity = {{0, 0}};
    double maximum_velocity = std::numeric_limits<double>::min();
    for (int stage_id = 0; stage_id < N; stage_id++) {
        Stage stage;
        std::cin >> stage;
        std::priority_queue<int64_t> existing_payloads;
        for (const auto& [mass, valocity] : mass2velocity)
            existing_payloads.push(mass);
        while (existing_payloads.size()) {
            int64_t payload = existing_payloads.top();
            existing_payloads.pop();
            int64_t full_mass = stage.TotalMass(payload);
            if ((stage.IsUseless(payload)) || (full_mass > 10000))
                continue;
            double full_velocity = mass2velocity[payload] + stage.ComputeVelocityAdd(payload);
            maximum_velocity = std::max(maximum_velocity, full_velocity);
            if (mass2velocity.count(full_mass))
                mass2velocity[full_mass] = std::max(mass2velocity[full_mass], full_velocity);
            else
                mass2velocity[full_mass] = full_velocity;
        }
    }

    std::cout << static_cast<int64_t>(std::round(maximum_velocity)) << "\n";
    return 0;
}
