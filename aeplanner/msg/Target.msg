geometry_msgs/Point position
uint8 type          # 0=U_TARGET, 1=E_OCC (occ/unk frontier), 2=E_FREE (free/unk frontier)
float32 priority
geometry_msgs/Vector3 normal
float32 w_normal    # PCA anisotropy; < 0.4 means unreliable normal
int32 fail_count
