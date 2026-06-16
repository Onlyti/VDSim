/* mfs_grid_eval — evaluate a CarMaker MF-Tyre/MF-Swift .tir over a (Fz, kappa,
 * alpha) grid and emit Fx/Fy/Mz, for the 3-way tire parity (VDSim · Chrono ·
 * CarMaker). See docs/design/CARMAKER_TIRE_PARITY.md.
 *
 * Build:
 *   CMI=/opt/ipg/carmaker/linux64-12.0.1
 *   gcc -I $CMI/include mfs_grid_eval.c -L $CMI/lib \
 *       -lmfswift_tire_interface -lm -Wl,-rpath,$CMI/lib -o mfs_grid_eval
 *
 * Run:
 *   ./mfs_grid_eval <tir> <grid.csv> <out.csv>
 *   grid.csv header: Fz,kappa,alpha[,gamma,...]  (extra columns ignored)
 *   out.csv:  Fz,kappa,alpha,Fx,Fy,Mz            (CarMaker MF-Swift, steady-state)
 *
 * NOTE: MF-Swift 2212 requires a modern MF6.x (.tir FITTYP >= 61) property file.
 * The legacy synthetic sample_pac02.tir (FITTYP 6 / MF5.2) is NOT accepted; use a
 * shared MF6.x file for the apples-to-apples 3-way (see the design doc).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mfs_tire_api.h"

static MFS_API_TIRE* g_tire;

/* Evaluate steady-state forces at a prescribed (Fz, kappa, alpha).
 * Fz is reached by a 1-D bisection on vertical penetration (Fz grows with it). */
static void eval_point(double Fz_tgt, double kappa, double alpha,
                       double R, double vx, double* Fx, double* Fy, double* Mz) {
    double pen_lo = 0.0, pen_hi = 0.08;   /* [m] vertical penetration search range */
    double force[3] = {0,0,0}, torque[3] = {0,0,0};
    for (int it = 0; it < 40; ++it) {
        const double pen = 0.5 * (pen_lo + pen_hi);
        MFS_INPUT_DATA in; memset(&in, 0, sizeof(in));
        in.time = (double)it * 1.0;        /* advance time so the internal solver steps */
        in.wheel_carrier_position_G[2] = R - pen;            /* wheel-center height */
        in.wheel_carrier_velocity_WC[0] = vx;                /* forward */
        in.wheel_carrier_velocity_WC[1] = vx * tan(alpha);   /* lateral → slip angle */
        in.WC_to_G_transformation[0] = 1; in.WC_to_G_transformation[4] = 1;
        in.WC_to_G_transformation[8] = 1;
        in.wheel_angular_velocity = vx * (1.0 + kappa) / R;  /* kappa = (wR-vx)/vx */
        mfs_api_tire_set_input(g_tire, &in);
        mfs_api_tire_update(g_tire);
        mfs_api_tire_get_output(g_tire, force, torque, NULL, 0);
        if (force[2] < Fz_tgt) pen_lo = pen; else pen_hi = pen;
    }
    *Fx = force[0]; *Fy = force[1]; *Mz = torque[2];
}

int main(int argc, char** argv) {
    if (argc < 4) { fprintf(stderr, "usage: %s <tir> <grid.csv> <out.csv>\n", argv[0]); return 2; }
    MFS_API_SIMULATION* sim = mfs_api_simulation_create();
    g_tire = mfs_api_tire_create(sim);
    if (!mfs_api_tire_initialize_tire_property_file(g_tire, argv[1])) {
        fprintf(stderr, "tir load failed\n"); return 1; }
    mfs_api_tire_initialize_road_type(g_tire, MFS_API_ROAD_TYPE_DEFAULT_FLAT);
    mfs_api_tire_initialize_contact_mode(g_tire, MFS_API_CONTACT_MODE_SMOOTH_ROAD);
    mfs_api_tire_initialize_dynamics_mode(g_tire, MFS_API_DYNAMICS_MODE_STEADY_STATE);
    mfs_api_tire_initialize_magic_formula_mode(g_tire, MFS_API_MF_MODE_COMBINED_LOADS);
    mfs_api_tire_initialize_tire_side(g_tire, MFS_API_TIRE_SIDE_LEFT);
    if (!mfs_api_tire_init(g_tire)) {
        fprintf(stderr, "tire_init failed — MF-Swift needs an MF6.x (.tir FITTYP>=61) file\n");
        return 1; }

    FILE* in = fopen(argv[2], "r"); if (!in) { perror("grid"); return 1; }
    FILE* out = fopen(argv[3], "w"); if (!out) { perror("out"); return 1; }
    fprintf(out, "Fz,kappa,alpha,Fx,Fy,Mz\n");

    char line[1024]; int header = 1, n = 0;
    const double R = 0.31, vx = 15.0;
    while (fgets(line, sizeof line, in)) {
        if (header) { header = 0; continue; }       /* skip CSV header */
        double Fz, kappa, alpha;
        if (sscanf(line, "%lf,%lf,%lf", &Fz, &kappa, &alpha) != 3) continue;
        double Fx, Fy, Mz;
        eval_point(Fz, kappa, alpha, R, vx, &Fx, &Fy, &Mz);
        fprintf(out, "%.3f,%.5f,%.5f,%.3f,%.3f,%.4f\n", Fz, kappa, alpha, Fx, Fy, Mz);
        ++n;
    }
    fclose(in); fclose(out);
    printf("wrote %d points -> %s\n", n, argv[3]);
    return 0;
}
