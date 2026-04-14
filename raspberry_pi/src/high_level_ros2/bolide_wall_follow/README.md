# bolide_wall_follow
This package provides several wall-following and navigation strategies for the vehicle.

### General information
The package includes implementations for PID wall following, RANSAC-based wall following, and LIDAR-only navigation. The potential-field based navigator has been moved to the separate `potential_field_nav` package.

## How it works
- Each node subscribes to `/scan` and publishes commands on `/cmd_vel` and `/cmd_dir` as needed.
- Launch files are provided to run the full stack including sensor and low-level nodes.

## Troubleshooting
- If a node does not start, ensure its console script is declared in `setup.py` and that the package is built.
- For navigation oscillations or unexpected behavior, check parameters in launch files and tune gains.
