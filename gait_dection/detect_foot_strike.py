right_foot_markers = (
    'RD2P',
    'RD5P',
    'RHEE',
    'RLATH',
    'RD5M',
    'RD1M',
    'RP1M',
)

left_foot_markers = (
    'LD2P',
    'LD5P',
    'LHEE',
    'LLATH',
    'LD5M',
    'LD1M',
    'LP1M',
)

markers = left_foot_markers + right_foot_markers
z_coords = {marker: [] for marker in markers}
y_coords = {marker: [] for marker in markers}
x_coords = {marker: [] for marker in markers}
z_velo = {marker: [] for marker in markers}
z_accel = {marker: [] for marker in markers}

for marker in markers:
    z_coords[marker].extend(vicon.GetTrajectoryAtFrame(subject, marker, frame)[2] for frame in range(user_defined_region[0], user_defined_region[1]))
    y_coords[marker].extend(vicon.GetTrajectoryAtFrame(subject, marker, frame)[1] for frame in range(user_defined_region[0], user_defined_region[1]))
    x_coords[marker].extend(vicon.GetTrajectoryAtFrame(subject, marker, frame)[0] for frame in range(user_defined_region[0], user_defined_region[1]))
    z_velo[marker].extend(np.gradient(z_coords[marker])) 
    z_accel[marker].extend(np.diff(z_coords[marker], 2))

def find_cycles(marker: str = 'RD2P'):
    foot_down_frames = []
    def is_z_accel_peak(i, threshold: float = 4):
        return marker_z_accel[i] > threshold and marker_z_accel[i-1] < marker_z_accel[i] > marker_z_accel[i+1]
 
    def is_z_velo_trough(i, threshold: float = -4):
        return marker_z_velo[i] < threshold and marker_z_velo[i-1] > marker_z_velo[i] < marker_z_velo[i+1]
    
    z_accel_peak = z_velo_trough = False
    marker_z_accel = z_accel[marker]
    marker_z_velo = z_velo[marker]
    marker_z_pos = z_coords[marker]
 
    for i in range(1, len(marker_z_accel) - 1):
        if is_z_accel_peak(i):
            z_accel_peak = True    

        if is_z_velo_trough(i):
            z_velo_trough = True

        if z_accel_peak and z_velo_trough and marker_z_accel[i-1] > marker_z_velo[i-1] and marker_z_accel[i] < marker_z_velo[i] and marker_z_pos[i] < 120:
            foot_down_frames.append(i + user_defined_region[0])
            z_accel_peak = z_velo_trough = False

    return foot_down_frames
