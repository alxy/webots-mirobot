from scipy.spatial.transform import Rotation as R

r = R.from_euler('xyz', [0, 0, 90], degrees=True)
print(r.as_matrix())