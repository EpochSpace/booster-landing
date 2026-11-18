// pdg_flight: solve 6-DoF powered descent guidance with SCvx, fly it closed
// loop through the pdg rigid-body simulator, and dump everything the 2D
// real-time viewer needs.
//
// Writes into --out DIR:
//   reference.csv  SCvx guidance nodes (the plan)
//   flight.csv     closed-loop flight log, sampled at --log-hz
//   meta.json      vehicle geometry, limits and touchdown statistics
//
// The flight log is taken from inside the command callback of RocketSim::fly,
// which the simulator calls exactly once per integrator step at the step start
// (zero-order hold), so every sample is a real controller tick.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "pdg/scvx.hpp"
#include "pdg/sim.hpp"

namespace {

constexpr double kDeg = 180.0 / 3.14159265358979323846;

struct Sample {
    double t;
    pdg::State x;
    pdg::Vec3 uCmd;   // commanded body-frame thrust [N]
};

// --key value command line, with vec3 parsed as "x,y,z"
struct Args {
    std::string out = "out";
    pdg::Vec3 r0 = {200.0, 100.0, 500.0};
    pdg::Vec3 v0 = {-30.0, 5.0, -50.0};
    double dt = 0.005;
    double logHz = 50.0;
    double tMax = 120.0;
    double thrustScale = 1.0;
    double ispScale = 1.0;
    double misalignDeg = 0.0;
    double misalignAzDeg = 0.0;
    double actuatorTau = 0.0;
    double aoaTrigger = 40.0;
    double aoaMaxDeg = 35.0;
    double glideSlopeDeg = 20.0;
    double tiltMaxDeg = 70.0;
    double gimbalMaxDeg = 12.0;
    bool quiet = false;

    // a preset supplies a whole vehicle; anything the user passed explicitly
    // still wins, so these record what was actually on the command line
    std::string preset = "default";
    bool setR0 = false, setV0 = false, setAoa = false;
    bool setGlide = false, setTilt = false, setGimbal = false;

    // vehicle and solver overrides; <= 0 means "keep the pdg default"
    double mWet = -1.0, mDry = -1.0, isp = -1.0;
    double thrustMin = -1.0, thrustMax = -1.0;
    double tfGuess = -1.0, tfMin = -1.0, tfMax = -1.0;
    int nodes = -1;
    double thrustMargin = 1.0;   // guidance plans to this fraction of Tmax
    // autopilot gains; <= 0 means "keep the pdg default"
    double posP = -1.0, posD = -1.0, attP = -1.0, attD = -1.0, maxCorrAccel = -1.0;
    double cutoffAlt = -1.0;     // engine cutoff altitude; <0 = decide from T/W
    // Cut as low as the state allows: touchdown speed after cutoff is
    // sqrt(v^2 + 2 g h), so an early trigger costs altitude. Below 1.5 m the
    // plan's own clock gets there first anyway.
    double mecoAlt = 1.5;        // below this, cut once the descent is arrested
    double mecoSpeed = 1.0;      // "arrested" means descending slower than this
};

bool parseVec3(const char* s, pdg::Vec3& v) {
    return std::sscanf(s, "%lf,%lf,%lf", &v[0], &v[1], &v[2]) == 3;
}

bool parseArgs(int argc, char** argv, Args& a) {
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto need = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", k.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (k == "--out") a.out = need();
        else if (k == "--r0") { if (!parseVec3(need(), a.r0)) return false; a.setR0 = true; }
        else if (k == "--v0") { if (!parseVec3(need(), a.v0)) return false; a.setV0 = true; }
        else if (k == "--preset") a.preset = need();
        else if (k == "--dt") a.dt = std::atof(need());
        else if (k == "--log-hz") a.logHz = std::atof(need());
        else if (k == "--t-max") a.tMax = std::atof(need());
        else if (k == "--thrust-scale") a.thrustScale = std::atof(need());
        else if (k == "--isp-scale") a.ispScale = std::atof(need());
        else if (k == "--misalign-deg") a.misalignDeg = std::atof(need());
        else if (k == "--misalign-az-deg") a.misalignAzDeg = std::atof(need());
        else if (k == "--actuator-tau") a.actuatorTau = std::atof(need());
        else if (k == "--aoa-trigger") { a.aoaTrigger = std::atof(need()); a.setAoa = true; }
        else if (k == "--aoa-max-deg") { a.aoaMaxDeg = std::atof(need()); a.setAoa = true; }
        else if (k == "--glide-slope-deg") { a.glideSlopeDeg = std::atof(need()); a.setGlide = true; }
        else if (k == "--tilt-max-deg") { a.tiltMaxDeg = std::atof(need()); a.setTilt = true; }
        else if (k == "--gimbal-max-deg") { a.gimbalMaxDeg = std::atof(need()); a.setGimbal = true; }
        else if (k == "--m-wet") a.mWet = std::atof(need());
        else if (k == "--m-dry") a.mDry = std::atof(need());
        else if (k == "--isp") a.isp = std::atof(need());
        else if (k == "--thrust-min") a.thrustMin = std::atof(need());
        else if (k == "--thrust-max") a.thrustMax = std::atof(need());
        else if (k == "--tf-guess") a.tfGuess = std::atof(need());
        else if (k == "--tf-min") a.tfMin = std::atof(need());
        else if (k == "--tf-max") a.tfMax = std::atof(need());
        else if (k == "--nodes") a.nodes = std::atoi(need());
        else if (k == "--thrust-margin") a.thrustMargin = std::atof(need());
        else if (k == "--pos-p") a.posP = std::atof(need());
        else if (k == "--pos-d") a.posD = std::atof(need());
        else if (k == "--att-p") a.attP = std::atof(need());
        else if (k == "--att-d") a.attD = std::atof(need());
        else if (k == "--max-corr-accel") a.maxCorrAccel = std::atof(need());
        else if (k == "--cutoff-alt") a.cutoffAlt = std::atof(need());
        else if (k == "--meco-alt") a.mecoAlt = std::atof(need());
        else if (k == "--meco-speed") a.mecoSpeed = std::atof(need());
        else if (k == "--quiet") a.quiet = true;
        else if (k == "--help" || k == "-h") {
            std::printf(
                "pdg_flight [options]\n"
                "  --out DIR              output directory (default: out)\n"
                "  --r0 x,y,z             initial position [m]\n"
                "  --v0 x,y,z             initial velocity [m/s]\n"
                "  --dt SEC               integrator / controller step (default 0.005)\n"
                "  --log-hz HZ            flight log rate (default 50)\n"
                "  --thrust-scale S       plant thrust error (1.0 = nominal)\n"
                "  --isp-scale S          plant Isp error\n"
                "  --misalign-deg D       fixed engine cant\n"
                "  --misalign-az-deg D    azimuth of that cant\n"
                "  --actuator-tau SEC     first-order TVC actuator lag\n"
                "  --aoa-trigger V        speed above which the AoA limit applies [m/s]\n"
                "  --aoa-max-deg D        angle-of-attack limit\n"
                "  --glide-slope-deg D    glide slope cone (<=0 disables)\n"
                "  --tilt-max-deg D       tilt limit\n"
                "  --gimbal-max-deg D     gimbal limit\n"
                "  --m-wet KG             wet mass (propellant = wet - dry)\n"
                "  --m-dry KG             dry mass\n"
                "  --isp SEC              specific impulse\n"
                "  --thrust-min N         min throttle setting [N]\n"
                "  --thrust-max N         max thrust [N]\n"
                "  --tf-guess SEC         initial guess for the time of flight\n"
                "  --tf-min SEC           lower bound on the time of flight\n"
                "  --tf-max SEC           upper bound on the time of flight\n"
                "  --nodes K              SCvx temporal nodes (default 21)\n"
                "  --quiet                suppress the SCvx iteration table\n");
            std::exit(0);
        }
        else {
            std::fprintf(stderr, "unknown option: %s\n", k.c_str());
            return false;
        }
    }
    return true;
}

double tiltDegOf(const double* q) {
    double c = pdg::quatTiltCos(q);
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    return std::acos(c) * kDeg;
}

}  // namespace

int main(int argc, char** argv) {
    Args a;
    if (!parseArgs(argc, argv, a)) return 2;

    pdg::ScvxParams P;
    P.verbose = !a.quiet;

    // ---- preset ----------------------------------------------------------
    // pdg's defaults are the textbook Mars case: a 15 t stage with T_min/W of
    // 0.82, i.e. one that can hover. A Falcon-9-class first stage cannot --
    // a single Merlin at minimum throttle already outweighs the empty stage --
    // and that changes the problem qualitatively, so it gets its own vehicle.
    if (a.preset == "f9") {
        // Published Falcon 9 Full Thrust stage-1 figures: 433.1 t gross,
        // 22.2 t empty, 410.9 t propellant (287.4 LOX + 123.5 RP-1), nine
        // Merlin 1D at 7607 kN sea level, Isp 282 s SL / 311 s vacuum.
        //
        // The landing-burn reserve is not published. Working it out from the
        // rocket equation for the final burn alone -- roughly 400 m/s against a
        // 25.6 t landed stage at 282 s -- gives about 4 t, so 4.5 t is the
        // reserve here. (Whole-return figures quoted online, around 55 t, are
        // the sum of boostback, entry and landing burns; the first two are
        // atmospheric and out of pdg's reach entirely.)
        P.rocket.mDry = 25600.0;        // landed stage, incl. legs and grid fins
        P.rocket.mWet = 30100.0;        // 4.5 t landing-burn reserve, 15 % of wet
        P.rocket.Isp  = 282.0;          // sea level, not the 300 s default
        P.rocket.Tmin = 0.40 * 845.0e3; // one Merlin, 40 % throttle floor
        P.rocket.Tmax = 845.0e3;
        // Pitch inertia from the same shell-plus-column model the donor bridge
        // uses, evaluated at the wet mass: 5.44e6 kg m^2, about 6.8x the pdg
        // default. Roll is the axial term and is not controllable anyway.
        P.rocket.J = {5.30e6, 5.30e6, 9.3e4};
        // Gimbal pivot sits roughly 2 m above the nozzle exit, and the centre of
        // mass runs 10.4 m (wet) to 12.4 m (dry) above the base; pdg holds this
        // arm constant, so take the middle.
        P.rocket.rT = {0.0, 0.0, -9.4};
        if (!a.setGimbal) P.gimbalMaxDeg = 5.0;     // Merlin TVC envelope
        if (!a.setTilt)   P.tiltMaxDeg   = 30.0;    // a landing burn stays upright
        if (!a.setGlide)  P.glideSlopeDeg = 20.0;
        P.omegaMaxDegS = 20.0;          // the donor logs 9.7 deg/s in terminal
        if (!a.setR0) a.r0 = {150.0, 0.0, 2000.0};
        if (!a.setV0) a.v0 = {-12.0, 0.0, -180.0};
        if (!a.setAoa) a.aoaTrigger = 0.0;          // no atmosphere to trip it
        if (a.tfGuess <= 0.0) P.tfGuess = 22.0;
        if (a.tfMax   <= 0.0) P.tfMax   = 45.0;
        if (a.nodes   <= 0)   P.K       = 31;
        // A minimum-fuel plan rides T_max for most of the burn, which leaves
        // the tracker nothing to correct with. Plan to a fraction of the
        // envelope and keep the rest as control authority.
        if (a.thrustMargin >= 1.0) a.thrustMargin = 0.85;
        // pdg's attitude gains assume its own 8e5 kg m^2 stage. This one has
        // 5.44e6 and a 5 deg gimbal, so the same error asks for 6.8x the
        // deflection and the loop sits on its stop. Size the loop to the
        // authority instead: with alpha_max = T sin(delta) L / J = 0.108 rad/s^2,
        // holding wn^2 * e_typ within that for a 5 deg error gives wn = 1.1 rad/s.
        if (a.attP <= 0.0) a.attP = 1.2;
        if (a.attD <= 0.0) a.attD = 2.9;   // zeta ~ 1.3, as the pdg defaults
    } else if (a.preset != "default") {
        std::fprintf(stderr, "unknown preset: %s (expected 'default' or 'f9')\n",
                     a.preset.c_str());
        return 2;
    }

    P.r0 = a.r0;
    P.v0 = a.v0;
    if (a.setGlide || a.preset == "default") P.glideSlopeDeg = a.glideSlopeDeg;
    if (a.setTilt  || a.preset == "default") P.tiltMaxDeg = a.tiltMaxDeg;
    if (a.setGimbal || a.preset == "default") P.gimbalMaxDeg = a.gimbalMaxDeg;
    if (a.aoaTrigger > 0.0)
        P.stcs.push_back(pdg::makeSpeedTriggeredAoA(a.aoaTrigger, a.aoaMaxDeg));

    if (a.mWet > 0.0) P.rocket.mWet = a.mWet;
    if (a.mDry > 0.0) P.rocket.mDry = a.mDry;
    if (a.isp > 0.0) P.rocket.Isp = a.isp;
    if (a.thrustMin > 0.0) P.rocket.Tmin = a.thrustMin;
    if (a.thrustMax > 0.0) P.rocket.Tmax = a.thrustMax;

    // the engine's real ceiling, before the planning margin is taken off it
    const double trueTmax = P.rocket.Tmax;
    P.rocket.Tmax *= a.thrustMargin;
    if (a.tfGuess > 0.0) P.tfGuess = a.tfGuess;
    if (a.tfMin > 0.0) P.tfMin = a.tfMin;
    if (a.tfMax > 0.0) P.tfMax = a.tfMax;
    if (a.nodes > 0) P.K = a.nodes;

    // The propellant budget is the whole story here: pdg models thrust and
    // gravity only, with no atmosphere, so nothing but the engine slows the
    // vehicle down. Print it next to the entry energy so an infeasible case is
    // obvious before the solver is blamed for it.
    {
        const pdg::RocketParams& R = P.rocket;
        const double dv = R.Isp * R.g0 * std::log(R.mWet / R.mDry);
        const double speed0 = std::sqrt(a.v0[0] * a.v0[0] + a.v0[1] * a.v0[1] +
                                        a.v0[2] * a.v0[2]);
        const double twr = R.Tmax / (R.mWet * R.g0);
        const double twrMinDry = R.Tmin / (R.mDry * R.g0);
        std::printf("preset : %s\n", a.preset.c_str());
        std::printf("vehicle: %.0f kg wet / %.0f kg dry (%.0f%% propellant), Isp %.0f s\n",
                    R.mWet, R.mDry, 100.0 * (R.mWet - R.mDry) / R.mWet, R.Isp);
        std::printf("         dV %.0f m/s, T/W %.2f max, %.2f min at dry mass -> %s\n",
                    dv, twr, twrMinDry,
                    twrMinDry > 1.0 ? "CANNOT hover, the burn is one shot"
                                    : "can hover");
        // energy bound: how fast it would arrive unpowered. The burn must cancel
        // at least that, plus whatever gravity takes back while it is burning.
        const double vArrival = std::sqrt(speed0 * speed0 + 2.0 * R.g0 * a.r0[2]);
        std::printf("entry  : %.0f m altitude, %.0f m/s -> unpowered arrival "
                    "%.0f m/s (dV floor)\n", a.r0[2], speed0, vArrival);
        if (vArrival > dv)
            std::printf("  warning: that floor already exceeds the dV budget -- "
                        "no drag is modelled, so this cannot close\n");
        if (twr <= 1.0)
            std::printf("  warning: max thrust cannot even hold the vehicle up\n");
    }

    pdg::ScvxPlanner planner(P);
    pdg::ScvxSolution sol;
    if (!planner.solve(sol)) {
        std::fprintf(stderr, "SCvx did not converge (%zu iterations)\n", sol.history.size());
        return 1;
    }

    std::printf("\n6-DoF SCvx guidance\n");
    std::printf("  time of flight : %7.3f s\n", sol.tf);
    std::printf("  fuel used      : %7.1f kg\n", sol.fuelUsed);
    std::printf("  SCvx iterations: %zu\n", sol.history.size());

    // ---- closed-loop flight, logged -------------------------------------
    pdg::State x0{};
    x0[pdg::iM] = P.rocket.mWet;
    for (int i = 0; i < 3; ++i) x0[pdg::iR + i] = P.r0[i];
    for (int i = 0; i < 3; ++i) x0[pdg::iV + i] = P.v0[i];
    for (int i = 0; i < 4; ++i) x0[pdg::iQ + i] = P.q0[i];
    for (int i = 0; i < 3; ++i) x0[pdg::iW + i] = P.w0[i];

    pdg::PlantDispersions disp;
    disp.thrustScale = a.thrustScale;
    disp.ispScale = a.ispScale;
    disp.thrustMisalignRad = a.misalignDeg / kDeg;
    disp.misalignAzimuthRad = a.misalignAzDeg / kDeg;
    disp.actuatorTau = a.actuatorTau;

    // The plant and the autopilot keep the full engine envelope; only the
    // planner was held back, so the difference is headroom the tracker can
    // spend correcting.
    pdg::RocketParams plantRocket = P.rocket;
    plantRocket.Tmax = trueTmax;
    pdg::RocketSim sim(plantRocket, disp);
    pdg::TrackingGains gains;
    if (a.posP > 0.0) gains.posP = a.posP;
    if (a.posD > 0.0) gains.posD = a.posD;
    if (a.attP > 0.0) gains.attP = a.attP;
    if (a.attD > 0.0) gains.attD = a.attD;
    if (a.maxCorrAccel > 0.0) gains.maxCorrAccel = a.maxCorrAccel;
    pdg::TrajectoryTracker tracker(plantRocket, sol, gains);

    std::vector<Sample> log;
    log.reserve(static_cast<size_t>(a.tMax * a.logHz) + 64);
    const double logDt = 1.0 / a.logHz;
    double nextLog = 0.0;

    // A stage that cannot hover has to shut the engine down: past the end of the
    // plan the tracker feeds forward -m*g, which is below T_min, so the clip to
    // T_min turns the last metre into a climb-away. Real vehicles cut at contact.
    const bool canHover = plantRocket.Tmin < plantRocket.mDry * plantRocket.g0;
    const double cutAlt = a.cutoffAlt >= 0.0 ? a.cutoffAlt : (canHover ? -1.0 : 0.0);
    const bool cutoffEnabled = cutAlt >= 0.0;
    double cutoffTime = -1.0;

    pdg::TouchdownState td = sim.fly(
        x0,
        [&](double t, const pdg::State& x) {
            pdg::Vec3 cmd{0.0, 0.0, 0.0};   // MECO: the free fall after cutoff
            // Touchdown detect, as a stage that cannot hover must have: cut when
            // the descent is arrested close to the deck, rather than waiting for
            // the plan's clock to run out somewhere above it. The plan's end and
            // a hard floor stay as backstops.
            const double alt = x[pdg::iR + 2];
            const double vz = x[pdg::iV + 2];
            const bool arrested = alt <= a.mecoAlt && vz > -a.mecoSpeed;
            const bool cut = cutoffEnabled &&
                             (arrested || t >= sol.tf || alt <= cutAlt);
            if (cut) {
                if (cutoffTime < 0.0) cutoffTime = t;
            } else {
                cmd = tracker.command(t, x);
            }
            // fly() calls the controller once per step at the step start, plus
            // one extra time at t=0 to prime the actuator lag: keep the first.
            // The post-cutoff samples are logged too, or the last metre of the
            // descent would be missing from the flight record.
            if (t + 1e-12 >= nextLog && (log.empty() || t > log.back().t + 1e-12)) {
                log.push_back({t, x, cmd});
                nextLog += logDt;
            }
            return cmd;
        },
        a.dt, a.tMax);

    if (cutoffTime >= 0.0)
        std::printf("  engine cutoff  : t = %.2f s, plan ends %.2f s -> free fall\n",
                    cutoffTime, sol.tf);
    std::printf("  touchdown      : lateral %.2f m, descent %.2f m/s, tilt %.2f deg, "
                "rate %.2f deg/s%s\n",
                td.lateralError, td.verticalSpeed, td.tiltDeg, td.rateDegS,
                td.landed ? "" : "   [NO TOUCHDOWN within --t-max]");

    // ---- outputs ---------------------------------------------------------
    std::error_code ec;
    std::filesystem::create_directories(a.out, ec);
    if (ec) {
        std::fprintf(stderr, "cannot create %s: %s\n", a.out.c_str(), ec.message().c_str());
        return 3;
    }

    const std::string refPath = a.out + "/reference.csv";
    const std::string flightPath = a.out + "/flight.csv";
    const std::string metaPath = a.out + "/meta.json";

    const char* header =
        "t,m,rx,ry,rz,vx,vy,vz,q0,q1,q2,q3,wx,wy,wz,Tx,Ty,Tz,Tmag,tilt_deg,speed\n";

    auto writeRow = [](std::ofstream& f, double t, const pdg::State& x, const double* u) {
        const double Tm = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
        const double spd = std::sqrt(x[pdg::iV] * x[pdg::iV] +
                                     x[pdg::iV + 1] * x[pdg::iV + 1] +
                                     x[pdg::iV + 2] * x[pdg::iV + 2]);
        f << t << ',' << x[pdg::iM];
        for (int i = 0; i < 3; ++i) f << ',' << x[pdg::iR + i];
        for (int i = 0; i < 3; ++i) f << ',' << x[pdg::iV + i];
        for (int i = 0; i < 4; ++i) f << ',' << x[pdg::iQ + i];
        for (int i = 0; i < 3; ++i) f << ',' << x[pdg::iW + i];
        for (int i = 0; i < 3; ++i) f << ',' << u[i];
        f << ',' << Tm << ',' << tiltDegOf(x.data() + pdg::iQ) << ',' << spd << '\n';
    };

    {
        std::ofstream f(refPath);
        if (!f) { std::fprintf(stderr, "cannot write %s\n", refPath.c_str()); return 3; }
        f.precision(10);
        f << header;
        for (size_t k = 0; k < sol.t.size(); ++k) writeRow(f, sol.t[k], sol.x[k], sol.u[k].data());
    }
    {
        std::ofstream f(flightPath);
        if (!f) { std::fprintf(stderr, "cannot write %s\n", flightPath.c_str()); return 3; }
        f.precision(10);
        f << header;
        for (const Sample& s : log) writeRow(f, s.t, s.x, s.uCmd.data());
    }
    {
        std::ofstream f(metaPath);
        if (!f) { std::fprintf(stderr, "cannot write %s\n", metaPath.c_str()); return 3; }
        f.precision(10);
        const pdg::RocketParams& R = P.rocket;
        f << "{\n"
          << "  \"preset\": \"" << a.preset << "\",\n"
          << "  \"vehicle\": {\n"
          << "    \"m_wet\": " << R.mWet << ", \"m_dry\": " << R.mDry << ",\n"
          << "    \"isp\": " << R.Isp << ", \"t_min\": " << R.Tmin
          << ", \"t_max\": " << trueTmax << ",\n"
          << "    \"t_max_planned\": " << R.Tmax << ",\n"
          << "    \"twr_min_at_dry\": " << R.Tmin / (R.mDry * R.g0) << ",\n"
          << "    \"inertia\": [" << R.J[0] << ", " << R.J[1] << ", " << R.J[2] << "],\n"
          << "    \"engine_offset_m\": " << -R.rT[2] << ",\n"
          << (a.preset == "f9"
                ? "    \"length_m\": 47.7, \"diameter_m\": 3.66\n"
                : "    \"length_m\": 41.2, \"diameter_m\": 3.7\n")
          << "  },\n"
          << "  \"limits\": {\n"
          << "    \"tilt_max_deg\": " << P.tiltMaxDeg << ", \"gimbal_max_deg\": " << P.gimbalMaxDeg << ",\n"
          << "    \"glide_slope_deg\": " << P.glideSlopeDeg << ", \"omega_max_deg_s\": " << P.omegaMaxDegS << ",\n"
          << "    \"aoa_trigger_mps\": " << a.aoaTrigger << ", \"aoa_max_deg\": " << a.aoaMaxDeg << "\n"
          << "  },\n"
          << "  \"dispersions\": {\n"
          << "    \"thrust_scale\": " << a.thrustScale << ", \"isp_scale\": " << a.ispScale << ",\n"
          << "    \"misalign_deg\": " << a.misalignDeg << ", \"actuator_tau\": " << a.actuatorTau << "\n"
          << "  },\n"
          << "  \"guidance\": {\n"
          << "    \"tf\": " << sol.tf << ", \"fuel_used_kg\": " << sol.fuelUsed
          << ", \"scvx_iterations\": " << sol.history.size() << "\n"
          << "  },\n"
          << "  \"touchdown\": {\n"
          << "    \"landed\": " << (td.landed ? "true" : "false") << ", \"time\": " << td.time << ",\n"
          << "    \"lateral_error_m\": " << td.lateralError << ", \"descent_rate_mps\": " << td.verticalSpeed << ",\n"
          << "    \"lateral_speed_mps\": " << td.lateralSpeed << ", \"tilt_deg\": " << td.tiltDeg << ",\n"
          << "    \"rate_deg_s\": " << td.rateDegS << ", \"propellant_used_kg\": " << td.propellantUsed << "\n"
          << "  },\n"
          << "  \"log\": { \"rate_hz\": " << a.logHz << ", \"samples\": " << log.size()
          << ", \"dt\": " << a.dt << " }\n"
          << "}\n";
    }

    std::printf("  wrote          : %s, %s, %s (%zu samples)\n",
                refPath.c_str(), flightPath.c_str(), metaPath.c_str(), log.size());
    return 0;
}
