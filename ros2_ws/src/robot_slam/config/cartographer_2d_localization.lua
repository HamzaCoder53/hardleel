-- Pure localization on a saved .pbstream (load it with -load_state_filename ... -load_frozen_state=true).
include "cartographer_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }
POSE_GRAPH.optimize_every_n_nodes = 20

return options
