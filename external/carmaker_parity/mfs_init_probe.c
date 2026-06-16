#include <stdio.h>
#include "mfs_tire_api.h"
static void logcb(int sev, const char* ctx, const char* msg){
    printf("  [MFS sev=%d] %s: %s\n", sev, ctx?ctx:"", msg?msg:"");
}
int main(void){
    MFS_API_SIMULATION* sim = mfs_api_simulation_create_with_logger(logcb);
    if(!sim){ printf("sim NULL\n"); return 1; }
    MFS_API_TIRE* t = mfs_api_tire_create(sim);
    printf("set tir: %d\n", mfs_api_tire_initialize_tire_property_file(t,
        "/home/ailab-12/git/VDSim/external/chrono_parity/sample_pac02.tir"));
    printf("sim_mode: %d\n", mfs_api_tire_initialize_simulation_mode(t, 1));
    printf("road: %d\n", mfs_api_tire_initialize_road_type(t, MFS_API_ROAD_TYPE_DEFAULT_FLAT));
    printf("contact: %d\n", mfs_api_tire_initialize_contact_mode(t, MFS_API_CONTACT_MODE_SMOOTH_ROAD));
    printf("dyn: %d\n", mfs_api_tire_initialize_dynamics_mode(t, MFS_API_DYNAMICS_MODE_STEADY_STATE));
    printf("mf: %d\n", mfs_api_tire_initialize_magic_formula_mode(t, MFS_API_MF_MODE_COMBINED_LOADS));
    printf("side: %d\n", mfs_api_tire_initialize_tire_side(t, MFS_API_TIRE_SIDE_LEFT));
    printf("init: %d\n", mfs_api_tire_init(t));
    return 0;
}
